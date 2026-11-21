import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import math
import tempfile
import time
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from slam import LocalSLAM, transform, relative_prediction
from lidar_guidance import sector_ranges, checkpoint_delta, motor_spin, spin_during_scan


def room():
    # Asymmetric room with an internal feature; enough geometry for all 3 DOF.
    x = np.linspace(-2, 3, 160)
    y = np.linspace(-1.5, 2, 120)
    return np.vstack((np.column_stack((x, np.full_like(x, -1.5))),
                      np.column_stack((x, np.full_like(x, 2))),
                      np.column_stack((np.full_like(y, -2), y)),
                      np.column_stack((np.full_like(y, 3), y)),
                      np.column_stack((np.linspace(.5, 1.3, 60), np.linspace(.1, .9, 60)))))


class SLAMTests(unittest.TestCase):
    def test_exploration_colors_only_rays_with_valid_endpoints(self):
        from lidar_exploration import ExplorationGrid
        grid=ExplorationGrid(resolution=.1)
        grid.update([0.05,.05],[[.55,.05]])
        self.assertIn((2,0),grid.observed)
        self.assertNotIn((5,0),grid.observed)
        self.assertNotIn((0,2),grid.observed)
        self.assertNotIn((6,0),grid.observed)
        grid.update([.05,.05],[[.25,.05]])
        self.assertNotIn((2,0),grid.observed)
        image,rect=grid.image();self.assertEqual(image.shape[2],4)
        self.assertGreater(rect[2],0)

    def test_calibration_retains_all_nonzero_raw_returns(self):
        from coind6.protocol import Point
        from lidar_calibration import raw_calibration_points
        points=[Point(0,50,1,False),Point(180,13000,1,False),Point(90,0,1,False)]
        result=raw_calibration_points(points,(.115,0))
        np.testing.assert_allclose(result,[[.165,0],[-12.885,0]],atol=1e-8)

    def test_spin_prediction_anchors_centre_and_rejects_contradictory_scan(self):
        world=room()
        model=LocalSLAM();model.update(world,1,np.zeros(3))
        actual=np.array([.01,0,.03]);sensor=transform(world-actual[:2],(0,0,-actual[2]))
        result=model.update(sensor,1.1,np.array([0,0,.03]),rotation_only=True)
        self.assertTrue(result['accepted'],result['note'])
        np.testing.assert_allclose(result['pose'][:2],[0,0],atol=1e-10)
        model=LocalSLAM();model.update(world,1,np.zeros(3))
        actual=np.array([.12,0,.03]);sensor=transform(world-actual[:2],(0,0,-actual[2]))
        result=model.update(sensor,1.1,np.array([0,0,.03]),rotation_only=True)
        self.assertFalse(result['accepted'])
        self.assertEqual(len(model.path),1)

    def test_short_gap_requires_continuous_small_odometry(self):
        model=LocalSLAM();model.update(room(),1,np.zeros(3))
        self.assertTrue(model.update(room(),2.3,np.zeros(3))['accepted'])
        self.assertFalse(model.update(room(),3.5,np.array([.2,0,0]))['accepted'])

    def test_rotation_after_long_gap_recovers_only_with_consistent_observations(self):
        model=LocalSLAM();world=room();model.update(world,1,np.zeros(3))
        turn=math.pi/2;sensor=transform(world,(0,0,-turn))
        result=model.update(sensor,8,np.array([0,0,turn]))
        self.assertTrue(result['accepted'],result['note'])
        self.assertIn('recuperado',result['note'])
        self.assertAlmostEqual(result['pose'][2],turn,delta=.03)
        model=LocalSLAM();model.update(world,1,np.zeros(3))
        self.assertFalse(model.update(sensor+10,8,np.array([0,0,turn]))['accepted'])

    def test_incremental_rotation_with_partial_visibility(self):
        model=LocalSLAM();world=room()
        for i in range(20):
            angle=math.radians(i*5)
            sensor=transform(world,(0,0,-angle));sensor=sensor[sensor[:,0]>0]
            result=model.update(sensor,1+i*.1,np.array([0.,0.,angle]),rotation_only=i>0)
            self.assertTrue(result['accepted'],f'{i}: {result["note"]}')
        self.assertAlmostEqual(result['pose'][2],math.radians(95),delta=.05)

    def test_spin_detection_decodes_motor_polarity_and_requires_whole_scan(self):
        geometry={'swap_lr':1,'motor_l':-1,'motor_r':1}
        self.assertTrue(motor_spin({'pl':80,'pr':80},geometry))
        history=[(1.0,None,True),(1.1,None,True),(1.2,None,True)]
        self.assertTrue(spin_during_scan(history,1.0,1.2))
        history[1]=(1.1,None,False)
        self.assertFalse(spin_during_scan(history,1.0,1.2))
        self.assertFalse(spin_during_scan([],1.0,1.2))
    def test_sector_ranges_and_missing_data(self):
        points=np.array([[1,0],[1,.1],[1,-.1],[-.5,0],[-.5,.05],[-.5,-.05],
                         [0,.4],[.05,.4],[-.05,.4]])
        result=sector_ranges(points)
        self.assertAlmostEqual(result['frente'],1)
        self.assertAlmostEqual(result['atrás'],.5)
        self.assertAlmostEqual(result['izquierda'],.4)
        self.assertIsNone(result['derecha'])
        self.assertTrue(all(value is None for value in sector_ranges([]).values()))

    def test_checkpoint_vector_uses_robot_heading_and_shortest_angle(self):
        delta=checkpoint_delta([0,0,math.pi/2],[0,1,-math.pi+.1])
        np.testing.assert_allclose(delta,[1,0,math.degrees(math.pi/2+.1)],atol=1e-8)
        with self.assertRaises(ValueError):checkpoint_delta([0,float('nan'),0],[0,0,0])

    def test_recovers_translation_and_rotation(self):
        model = LocalSLAM(); world = room()
        self.assertTrue(model.update(world, 1)['accepted'])
        pose = np.array([.09, -.04, math.radians(2)])
        sensor = transform(world-pose[:2], (0, 0, -pose[2]))
        result = model.update(sensor, 1.1)
        self.assertTrue(result['accepted'], result['note'])
        np.testing.assert_allclose(result['pose'], pose, atol=.025)

    def test_rejects_outliers_and_gap_without_changing_map(self):
        model = LocalSLAM(); model.update(room(), 1)
        before = model.map.copy()
        self.assertFalse(model.update(room()+100, 1.1)['accepted'])
        self.assertFalse(model.update(room(), 3)['accepted'])
        np.testing.assert_array_equal(before, model.map)
        self.assertEqual(len(model.path), 1)

    def test_rejects_degenerate_wall(self):
        model = LocalSLAM()
        points = np.column_stack((np.linspace(-2, 2, 100), np.ones(100)))
        model.update(points, 1)
        self.assertFalse(model.update(points, 1.1)['accepted'])

    def test_odometry_delta_rotates_into_map_frame(self):
        pose = np.array([2., 3., math.pi/2])
        predicted = relative_prediction(pose, np.zeros(3), np.array([1., 0., .1]))
        np.testing.assert_allclose(predicted, [2, 4, math.pi/2+.1])


class WorkspaceTests(unittest.TestCase):
    def test_automatic_return_starts_one_routine_and_stops_on_finish(self):
        from app import Window
        from lidar_workspace import LidarWorkspace
        from unittest.mock import Mock
        with patch.object(LidarWorkspace, 'load'):
            panel = LidarWorkspace()
        panel.timer.stop()
        panel.odom.setChecked(True)
        panel.odom_ready = panel.lidar_fresh = True
        panel.calibration_time = time.monotonic()
        window = Mock()
        window.engine.motion_block.return_value = ''
        for name in ('primitive','live','routine','routine_running'):
            setattr(window.engine,name,None)
        window.cap.value.return_value = 85
        window.move_encoder.currentText.return_value = 'Left'
        panel.trial_motion_requested.connect(lambda mode: Window.run_lidar_trial(window,mode))
        panel.stop_requested.connect(window.stop)
        panel.trial_mode.setCurrentIndex(2)
        with tempfile.TemporaryDirectory() as tmp, patch('lidar_workspace.__file__', str(Path(tmp)/'module.py')):
            panel.trial_button.click()
            window.engine.post.assert_called_once_with('routine',([('wait',2),('distance',30),('wait',1),('distance',-30)],85,'left'))
            panel.trial_button.click()
            window.stop.assert_called_once()
        panel.shutdown()

    def test_manual_trial_records_telemetry_and_finishes_on_timeout(self):
        import json
        from lidar_workspace import LidarWorkspace
        with patch.object(LidarWorkspace, 'load'):
            panel = LidarWorkspace()
        panel.timer.stop()
        panel.odom.setChecked(True)
        state = dict(fresh=True, imu_valid=True, single_left=(0,0,0), telemetry=dict(tl=7,tr=9))
        panel.receive_state(state)
        panel.lidar_fresh = True
        with tempfile.TemporaryDirectory() as tmp, patch('lidar_workspace.__file__', str(Path(tmp)/'module.py')):
            panel.toggle_trial()
            panel.receive_state(state)
            panel.trial_start -= 31
            panel.poll()
            records = [json.loads(line) for line in panel.trial_path.read_text(encoding='utf8').splitlines()]
            self.assertIsNone(panel.trial_file)
            self.assertEqual(records[0]['event'], 'inicio')
            self.assertEqual(records[-1]['ticks'], [7,9])
        panel.shutdown()

    @classmethod
    def setUpClass(cls):
        from PySide6 import QtWidgets
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_calibration_sliders_preview_and_apply_same_sector(self):
        from lidar_workspace import LidarWorkspace
        with patch.object(LidarWorkspace,'load'):panel=LidarWorkspace()
        panel.timer.stop();panel.robot_stationary=True;panel.lidar_fresh=True
        panel.sector_center.setValue(0);panel.sector_width.setValue(180);panel.reset_map(save=False)
        panel.inspection_time=time.monotonic()
        panel.inspection_points=np.array([[1.115,0],[-.885,0],[.115,1],[.115,-1]])
        panel.calibration_time=panel.inspection_time;panel.calibration_points=panel.inspection_points.copy()
        with tempfile.TemporaryDirectory() as tmp:
            panel.config=Path(tmp)/'lidar.json';panel.begin_front_calibration();wizard=panel.front_wizard
            self.assertEqual(wizard.width.value(),180)
            self.assertIn(1.115,wizard.scan.getData()[0])
            self.assertIsNone(wizard.excluded_scan.getData()[0])
            wizard.choose_point(1.115,0);wizard.next_button.click()
            wizard.center.setValue(0);wizard.width.setValue(180)
            self.assertIn('incluye el frente',wizard.status.text())
            self.assertIn(1.115,wizard.excluded_scan.getData()[0])
            wizard.center_slider.setValue(180);wizard.width_slider.setValue(120)
            self.assertIn(1.115,wizard.scan.getData()[0])
            self.assertIn(-.885,wizard.excluded_scan.getData()[0])
            self.assertEqual(panel.mask[1:],(0,180))
            wizard.still.setChecked(True);wizard.width.setValue(360)
            self.assertFalse(wizard.apply_button.isEnabled())
            wizard.width.setValue(120);wizard.apply_button.click();self.app.processEvents()
            self.assertEqual(panel.mask[1:],(180,120))
        panel.shutdown();panel.close()

    def test_mount_and_pose_reset_without_hardware(self):
        from lidar_workspace import LidarWorkspace
        with patch.object(LidarWorkspace, 'load'):
            panel = LidarWorkspace()
        panel.timer.stop()
        panel.odom_mode.setCurrentIndex(2)
        panel.odom.setChecked(True)
        old = panel.generation
        panel.receive_state(dict(pose_revision=1, fresh=True, x=1000, y=2000, yaw=90))
        self.assertGreater(panel.generation, old)
        np.testing.assert_allclose(panel.history[-1][1], [1, 2, math.pi/2])
        panel.model.update(room(), 1)
        panel.receive_state(dict(pose_revision=2, fresh=False))
        self.assertTrue(panel.reference_lost)
        self.assertGreater(len(panel.model.map), 0)
        panel.source.setCurrentIndex(1); panel.reset_map(save=False)
        self.assertFalse(panel.use_odom)
        panel.shutdown()

    def test_inspection_applies_selected_mask_without_disabling_imu(self):
        from PySide6 import QtCore
        from lidar_workspace import LidarWorkspace
        with patch.object(LidarWorkspace,'load'):panel=LidarWorkspace()
        panel.timer.stop();panel.odom.setChecked(True)
        panel.sector_center.setValue(180);panel.sector_width.setValue(120)
        panel.reset_map(save=False)
        self.assertTrue(panel.use_odom);self.assertEqual(panel.mask[1:],(180,120))
        panel.show_inspection(True)
        self.assertFalse(panel.inspection_plot.isHidden());self.assertTrue(panel.plot.isHidden())
        scene=panel.inspection_plot.getViewBox().mapViewToScene(QtCore.QPointF(1.115,0))
        panel.inspection_cursor_moved((scene,))
        self.assertIn('0.0°',panel.angle_readout.text())
        panel.show_inspection(False);self.assertTrue(panel.inspection_plot.isHidden())
        panel.shutdown()

    def test_front_calibration_uses_selected_return_and_starts_new_map(self):
        from PySide6 import QtCore, QtTest
        from lidar_workspace import LidarWorkspace
        with patch.object(LidarWorkspace,'load'):panel=LidarWorkspace()
        panel.timer.stop();panel.robot_stationary=True;panel.lidar_fresh=True;panel.show()
        panel.inspection_time=time.monotonic();panel.inspection_points=np.array([[-.89,0.]])
        panel.calibration_time=panel.inspection_time;panel.calibration_points=panel.inspection_points.copy()
        with tempfile.TemporaryDirectory() as tmp:
            panel.config=Path(tmp)/'lidar.json';generation=panel.generation
            panel.begin_front_calibration();wizard=panel.front_wizard
            self.assertIsNotNone(wizard);self.app.processEvents()
            scene=wizard.plot.getViewBox().mapViewToScene(QtCore.QPointF(-.89,0))
            point=wizard.plot.mapFromScene(scene)
            QtTest.QTest.mouseClick(wizard.plot.viewport(),QtCore.Qt.MouseButton.LeftButton,pos=point)
            self.assertIsNotNone(wizard.candidate)
            self.assertEqual(panel.angle.value(),0)
            self.assertEqual(panel.generation,generation)
            wizard.choose_point(1.115,0)
            self.assertAlmostEqual(abs(wizard.candidate),180)
            wizard.next_button.click()
            wizard.still.setChecked(True);self.assertTrue(wizard.apply_button.isEnabled())
            wizard.apply_button.click();self.app.processEvents()
            self.assertAlmostEqual(abs(panel.angle.value()),180)
            self.assertEqual(panel.mask[1:],(180,0))
            self.assertGreater(panel.generation,generation)
        panel.shutdown();panel.close()

    def test_calibration_fit_live_and_refresh(self):
        from lidar_calibration import LidarCalibrationWizard
        wizard=LidarCalibrationWizard([[9.,0.],[-1.,1.]],(.115,0,25,.1,12),sector_center=0,sector_width=180)
        wizard.show();self.app.processEvents();wizard.fit_points()
        x_range,_=wizard.plot.getViewBox().viewRange()
        self.assertLessEqual(x_range[0],-1);self.assertGreaterEqual(x_range[1],9)
        wizard.update_live_points([[2.,0.],[3.,0.]])
        self.assertEqual(len(wizard.scan.getData()[0]),2)
        wizard.choose_point(2.,0.);self.assertFalse(wizard.live)
        frozen=wizard.points.copy();wizard.update_live_points([[7.,0.]])
        np.testing.assert_array_equal(wizard.points,frozen)
        wizard.next_button.click();self.assertEqual(wizard.stage,1)
        self.assertIn('confirma',wizard.requirements.text())
        wizard.still.setChecked(True);self.assertTrue(wizard.apply_button.isEnabled())
        wizard.replace_points([[2.,0.]])
        self.assertEqual(wizard.stage,0);self.assertIsNone(wizard.candidate)
        self.assertFalse(wizard.still.isChecked());self.assertEqual(len(wizard.scan.getData()[0]),1)
        wizard.close()

    def test_calibration_direction_matches_mapper_and_screen_x(self):
        from coind6.protocol import Point
        from lidar_calibration import LidarCalibrationWizard, raw_calibration_points
        from lidar_filters import partition_scan
        points=[Point(angle,1000,1,False) for angle in (0,90,180,270)]
        origin=(.115,0.)
        for index in range(4):
            raw=raw_calibration_points(points,origin)
            wizard=LidarCalibrationWizard(raw,(*origin,0.,0.,12.))
            wizard.show();self.app.processEvents()
            view=wizard.plot.getViewBox()
            left=view.mapViewToScene(__import__('PySide6').QtCore.QPointF(0,0)).x()
            right=view.mapViewToScene(__import__('PySide6').QtCore.QPointF(1,0)).x()
            self.assertLess(left,right)
            below=view.mapViewToScene(__import__('PySide6').QtCore.QPointF(0,0)).y()
            above=view.mapViewToScene(__import__('PySide6').QtCore.QPointF(0,1)).y()
            self.assertLess(above,below)
            wizard.choose_point(*raw[index])
            usable,_=partition_scan(points,(*origin,wizard.candidate,.05,12),0,180,0)
            np.testing.assert_allclose(usable[index],[1.115,0],atol=1e-8)
            wizard.close()

    def test_sector_only_keeps_current_orientation(self):
        from lidar_calibration import LidarCalibrationWizard
        wizard=LidarCalibrationWizard([[1.115,0]],(.115,0,0,0,12),current_angle=37,sector_center=170,sector_width=80)
        wizard.keep_orientation()
        self.assertEqual(wizard.stage,1);self.assertEqual(wizard.candidate,37)
        self.assertEqual((wizard.center.value(),wizard.width.value()),(170,80))
        wizard.close()

    def test_manual_sensor_rotation_matches_mapper_and_is_applied_only_on_save(self):
        from coind6.protocol import Point
        from lidar_calibration import raw_calibration_points
        from lidar_filters import partition_scan
        from lidar_workspace import LidarWorkspace
        from PySide6 import QtCore
        points=[Point(90,1000,1,False),Point(0,1000,1,False)]
        with patch.object(LidarWorkspace,'load'):panel=LidarWorkspace()
        panel.timer.stop();panel.robot_stationary=True;panel.lidar_fresh=True
        panel.calibration_time=time.monotonic();panel.calibration_points=raw_calibration_points(points,(.11,0))
        with tempfile.TemporaryDirectory() as tmp:
            panel.config=Path(tmp)/'lidar.json';panel.begin_front_calibration();wizard=panel.front_wizard
            wizard.angle_slider.setValue(900)
            self.assertTrue(wizard.live);self.assertAlmostEqual(wizard.candidate,90)
            drawn=np.column_stack(wizard.scan.getData())
            expected,_=partition_scan(points,(.11,0,90,.05,12),0,180,0)
            np.testing.assert_allclose(drawn,expected,atol=1e-8)
            self.assertEqual(panel.mount[2],0)
            wizard.next_button.click();wizard.still.setChecked(True);wizard.apply_button.click()
            self.app.processEvents();self.assertAlmostEqual(panel.mount[2],90)
        panel.shutdown();panel.close()

    def test_robot_draw_follows_pose_and_keeps_sensor_offset(self):
        from lidar_workspace import LidarWorkspace
        with patch.object(LidarWorkspace,'load'):panel=LidarWorkspace()
        panel.timer.stop();panel.draw_robot(np.array([1.,2.,math.pi/2]))
        tail, tip = np.column_stack(panel.robot_direction.getData())[:2]
        np.testing.assert_allclose(tip-tail, [0., -.16], atol=1e-8)
        x,y=panel.robot_body_item.getData()
        np.testing.assert_allclose([x[1],y[1]],[1.115,2.115],atol=1e-8)
        x,y=panel.robot_sensor_item.getData()
        np.testing.assert_allclose([x[0],y[0]],[1,2.11],atol=1e-8)
        panel.shutdown()

    def test_checkpoint_requires_recent_live_accepted_localization(self):
        from unittest.mock import Mock
        from lidar_workspace import LidarWorkspace
        with patch.object(LidarWorkspace,'load'):panel=LidarWorkspace()
        panel.timer.stop()
        panel.live=True;panel.reader=Mock();panel.reader.is_alive.return_value=True
        panel.lidar_fresh=True;panel.robot_stationary=True;panel.result={'accepted':True,'pose':np.array([1.,2.,0.]),'path':np.zeros((2,3))}
        panel.result_time=time.monotonic()-2
        panel.save_checkpoint();self.assertIsNone(panel.checkpoint)
        panel.result_time=time.monotonic();panel.save_checkpoint()
        np.testing.assert_allclose(panel.checkpoint,[1,2,0])
        panel.result={'accepted':False,'pose':np.array([2.,2.,0.])}
        panel.update_checkpoint();self.assertIn('no disponible',panel.checkpoint_status.text())
        panel.reset_map(save=False);self.assertIsNone(panel.checkpoint)
        panel.reader=None;panel.shutdown()

    def test_acquisition_to_map_and_export(self):
        from lidar_workspace import LidarWorkspace
        from coind6.acquisition import Acquisition, DemoSource
        with patch.object(LidarWorkspace, 'load'):
            panel = LidarWorkspace()
        panel.timer.stop(); panel.live = False
        panel.reader = Acquisition(DemoSource()); panel.reader.start()
        try:
            deadline = time.monotonic()+4
            while panel.result is None and time.monotonic() < deadline:
                panel.poll(); self.app.processEvents(); time.sleep(.02)
            self.assertIsNotNone(panel.result)
            self.assertTrue(panel.result['accepted'])
            with tempfile.TemporaryDirectory() as tmp:
                destination = str(Path(tmp)/'map.npz')
                with patch('lidar_workspace.QtWidgets.QFileDialog.getSaveFileName', return_value=(destination, '')):
                    panel.export()
                with np.load(destination) as saved:
                    self.assertEqual(saved['map_xy_m'].shape[1], 2)
                    np.testing.assert_allclose(saved['mount_xy_yaw_deg'], [.11, 0, 0])
                    self.assertEqual(str(saved['source']), 'replay')
                    self.assertIn('created_utc',str(saved['metadata_json']))
                    self.assertEqual(saved['observed_cells_xy'].shape[1],2)
                self.assertTrue(Path(destination).with_suffix('.png').is_file())
        finally:
            panel.shutdown()
        self.assertFalse(panel.reader.is_alive())

    def test_finish_waits_for_pending_scan_and_keeps_map_on_reset(self):
        from concurrent.futures import Future
        from lidar_workspace import LidarWorkspace
        with patch.object(LidarWorkspace,'load'):
            panel=LidarWorkspace()
        panel.timer.stop();panel.odom.setChecked(True)
        future=Future();panel.future=future;panel.job_generation=panel.generation
        result=panel.model.update(room(),1)
        panel.job_scan_points=room()
        with patch.object(panel,'export') as export:
            panel.finish_scan();panel.poll();export.assert_not_called()
            future.set_result(result);panel.poll();export.assert_called_once()
        self.assertEqual(len(panel.scan_item.getData()[0]),len(room()))
        self.assertGreater(len(panel.scan_item.getData()[0]),len(result['points']))
        self.assertIs(panel.result,result)
        panel.receive_state(dict(pose_revision=999,fresh=False))
        self.assertIs(panel.result,result)
        self.assertTrue(panel.finished)
        panel.shutdown()

    def test_scan_preview_updates_while_slam_worker_is_busy(self):
        from concurrent.futures import Future
        from unittest.mock import Mock
        from queue import Queue
        from coind6.protocol import Point
        from coind6.scans import Scan
        from lidar_workspace import LidarWorkspace
        with patch.object(LidarWorkspace,'load'):panel=LidarWorkspace()
        panel.timer.stop();panel.live=True;panel.future=Future()
        now=time.monotonic();scan=Scan(1,(Point(0,1000,100,False),),now-.1,now,'ring',False)
        panel.reader=Mock();panel.reader.is_alive.return_value=True;panel.reader.messages=Queue()
        panel.reader.snapshot.return_value=(scan,{'last_rx':now,'connected':True,'state':'ready','packets':1,'bad_crc':0,'scans':1})
        panel.poll()
        self.assertEqual(len(panel.inspection_points),1)
        self.assertEqual(panel.inspection_time,now)
        panel.future=None;panel.reader=None;panel.shutdown()

    def test_rejected_map_clears_stale_scan_and_reports_lidar_separately(self):
        from concurrent.futures import Future
        from unittest.mock import Mock
        from queue import Queue
        from lidar_workspace import LidarWorkspace
        with patch.object(LidarWorkspace,'load'):panel=LidarWorkspace()
        panel.timer.stop();panel.live=True
        panel.scan_item.setData([1,2],[1,2]);panel.future=Future();panel.job_generation=panel.generation
        result=panel.model.result(False,'escaneo rechazado',np.empty((0,2)))
        panel.future.set_result(result)
        now=time.monotonic();panel.calibration_time=now
        panel.reader=Mock();panel.reader.is_alive.return_value=True;panel.reader.messages=Queue()
        panel.reader.snapshot.return_value=(None,{'last_rx':now,'connected':True,'state':'ready','packets':1,'bad_crc':0,'scans':1})
        panel.poll()
        self.assertIsNone(panel.scan_item.getData()[0])
        self.assertIn('recibiendo escaneos',panel.tracking_status.text())
        self.assertIn('sin localización',panel.tracking_status.text())
        panel.reader=None;panel.shutdown()

    def test_single_encoder_requires_imu_and_resets_on_discontinuity(self):
        from lidar_workspace import LidarWorkspace
        with patch.object(LidarWorkspace, 'load'):
            panel = LidarWorkspace()
        panel.timer.stop(); panel.odom.setChecked(True)
        state=dict(pose_revision=1, imu_segment=0, fresh=True, imu_valid=True,
                   single_left=(1000,2000,.3), single_right=(3000,4000,.4))
        panel.receive_state(state)
        np.testing.assert_allclose(panel.history[-1][1], [1,2,.3])
        panel.model.update(room(),1)
        panel.receive_state(dict(state,imu_segment=1,imu_valid=False))
        self.assertFalse(panel.history)
        self.assertTrue(panel.reference_lost)
        self.assertGreater(len(panel.model.map),0)
        panel.odom_mode.setCurrentIndex(1)
        panel.receive_state(dict(state,imu_segment=1))
        np.testing.assert_allclose(panel.history[-1][1], [3,4,.4])
        self.assertFalse(hasattr(panel,'serial'))
        self.assertEqual(panel.source.count(),2)
        panel.shutdown()


if __name__ == '__main__':
    unittest.main()
