"""Interactive, read-only preview of LiDAR mounting calibration."""
import math
import numpy as np
from PySide6 import QtCore, QtWidgets
import pyqtgraph as pg
from lidar_filters import exclusion_mask
from coind6.scans import cartesian


def raw_calibration_points(points,origin):
    """All nonzero sensor returns, with no saved angular or distance filters."""
    raw=np.asarray(cartesian(points,0.,True,0.,float('inf')),dtype=float).reshape(-1,3)
    return raw[:,:2]+np.asarray(origin,dtype=float)


class PickPlot(pg.PlotWidget):
    point_selected=QtCore.Signal(float,float)
    def mousePressEvent(self,event):
        if event.button()==QtCore.Qt.MouseButton.LeftButton:
            scene=self.mapToScene(event.position().toPoint())
            view=self.getViewBox()
            if view.sceneBoundingRect().contains(scene):
                pos=view.mapSceneToView(scene)
                self.point_selected.emit(pos.x(),pos.y())
                event.accept();return
        super().mousePressEvent(event)


class LidarCalibrationWizard(QtWidgets.QDialog):
    def __init__(self,points,mount,parent=None,sector_center=180.,sector_width=0.,body_margin=.02,current_angle=0.):
        super().__init__(parent)
        self.setWindowTitle('Asistente de calibración LiDAR');self.resize(850,650)
        self.points=np.asarray(points,dtype=float).reshape(-1,2).copy()
        self.mount=tuple(mount);self.candidate=None;self.selected=None;self.body_margin=body_margin;self.stage=0;self.live=True;self.current_angle=current_angle
        root=QtWidgets.QVBoxLayout(self)
        self.instructions=QtWidgets.QLabel()
        self.instructions.setWordWrap(True);root.addWidget(self.instructions)
        self.plot=PickPlot(title='Vista 360° · sin orientar: selecciona el objeto situado delante')
        self.plot.setAspectLocked(True);self.plot.showGrid(x=True,y=True)
        self.plot.setLabel('bottom','X (m)');self.plot.setLabel('left','Y (m)')
        root.addWidget(self.plot,1)
        self.circle=self.plot.plot(pen=pg.mkPen('#657583',width=1))
        self.mask_outline=self.plot.plot(pen=pg.mkPen('#ff9955',width=2))
        self.scan=self.plot.plot(pen=None,symbol='o',symbolSize=5,symbolBrush='#72eed6')
        self.excluded_scan=self.plot.plot(pen=None,symbol='o',symbolSize=6,symbolBrush='#ff9955')
        self.body=self.plot.plot(pen=pg.mkPen('#ff875e',width=2))
        self.sensor=self.plot.plot(pen=None,symbol='+',symbolSize=14,symbolPen=pg.mkPen('#ffd76a',width=2))
        self.selection=self.plot.plot(pen=None,symbol='o',symbolSize=12,symbolPen=pg.mkPen('#ffda69',width=2),symbolBrush=None)
        self.plot.point_selected.connect(self.choose_point)
        view_controls=QtWidgets.QHBoxLayout();root.addLayout(view_controls)
        fit=QtWidgets.QPushButton('Mostrar todos los puntos');fit.clicked.connect(self.fit_points);view_controls.addWidget(fit)
        zoom_in=QtWidgets.QPushButton('Acercar +');zoom_in.clicked.connect(lambda:self.zoom(.6));view_controls.addWidget(zoom_in)
        zoom_out=QtWidgets.QPushButton('Alejar −');zoom_out.clicked.connect(lambda:self.zoom(1.6));view_controls.addWidget(zoom_out)
        self.refresh_button=QtWidgets.QPushButton('Reanudar vista en vivo');view_controls.addWidget(self.refresh_button)
        self.status=QtWidgets.QLabel('Selecciona el termo delante. Amarillo: LiDAR.')
        self.status.setWordWrap(True);root.addWidget(self.status)
        self.pages=QtWidgets.QStackedWidget();root.addWidget(self.pages)
        orientation=QtWidgets.QWidget();orientation_layout=QtWidgets.QVBoxLayout(orientation)
        orientation_layout.setContentsMargins(0,0,0,0)
        hint=QtWidgets.QLabel('Con el AGV quieto, coloca objetos delante y a la izquierda y ajusta el giro hasta que coincidan con el dibujo: frente a la derecha, izquierda arriba.')
        hint.setWordWrap(True);orientation_layout.addWidget(hint);self.pages.addWidget(orientation)
        angle_row=QtWidgets.QHBoxLayout();orientation_layout.addLayout(angle_row)
        angle_row.addWidget(QtWidgets.QLabel('Giro del LiDAR respecto al AGV'))
        self.angle_slider=QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal);self.angle_slider.setRange(-1800,1800)
        self.angle_spin=QtWidgets.QDoubleSpinBox();self.angle_spin.setRange(-180,180);self.angle_spin.setDecimals(1);self.angle_spin.setSingleStep(.5);self.angle_spin.setSuffix('°');self.angle_spin.setValue(current_angle)
        self.angle_slider.setValue(round(current_angle*10));angle_row.addWidget(self.angle_slider,1);angle_row.addWidget(self.angle_spin)
        self.angle_slider.valueChanged.connect(lambda value:self.angle_spin.setValue(value/10))
        self.angle_spin.valueChanged.connect(self.adjust_angle)
        preview=QtWidgets.QPushButton('Previsualizar giro indicado')
        preview.clicked.connect(lambda:self.adjust_angle(self.angle_spin.value()));orientation_layout.addWidget(preview)
        keep=QtWidgets.QPushButton('Usar orientación actual y ajustar solo el sector')
        keep.clicked.connect(self.keep_orientation);orientation_layout.addWidget(keep)
        flip=QtWidgets.QPushButton('Probar montaje y máscara a 180° del ajuste guardado')
        flip.clicked.connect(lambda:self.preview_opposite_mount(sector_center,sector_width))
        orientation_layout.addWidget(flip)
        mask_page=QtWidgets.QWidget();mask_layout=QtWidgets.QVBoxLayout(mask_page)
        mask_layout.setContentsMargins(0,0,0,0);form=QtWidgets.QFormLayout();mask_layout.addLayout(form)
        self.center,self.center_slider=self.sector_control(form,'Centro excluido (°): 0 frente, ±180 atrás',-180,180,sector_center)
        self.width,self.width_slider=self.sector_control(form,'Ancho excluido (°): 0 ninguno, 360 todos',0,360,sector_width)
        reset=QtWidgets.QPushButton('Ver todos los retornos (ancho 0°)')
        reset.clicked.connect(lambda:self.width.setValue(0));mask_layout.addWidget(reset)
        note=QtWidgets.QLabel('El círculo representa 360°, no lecturas inventadas: las zonas sin retorno o tapadas siguen sin datos. Los puntos del chasis también se excluyen.')
        note.setWordWrap(True);mask_layout.addWidget(note)
        self.still=QtWidgets.QCheckBox('El AGV permaneció quieto durante la captura')
        mask_layout.addWidget(self.still);self.pages.addWidget(mask_page)
        self.requirements=QtWidgets.QLabel();self.requirements.setWordWrap(True);mask_layout.addWidget(self.requirements)
        buttons=QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        self.back_button=buttons.addButton('Atrás',QtWidgets.QDialogButtonBox.ButtonRole.ActionRole)
        self.next_button=buttons.addButton('Siguiente: zona excluida',QtWidgets.QDialogButtonBox.ButtonRole.ActionRole)
        self.apply_button=buttons.addButton('Guardar y crear mapa nuevo',QtWidgets.QDialogButtonBox.ButtonRole.AcceptRole)
        self.back_button.clicked.connect(lambda:self.set_stage(0));self.next_button.clicked.connect(lambda:self.set_stage(1))
        self.apply_button.setEnabled(False);self.still.toggled.connect(self.update_apply)
        buttons.accepted.connect(self.accept);buttons.rejected.connect(self.reject);root.addWidget(buttons)
        self.center.valueChanged.connect(self.redraw);self.width.valueChanged.connect(self.redraw)
        self.width.valueChanged.connect(self.update_apply)
        self.set_stage(0);self.fit_points()

    def set_stage(self,stage):
        if stage==1 and self.candidate is None:
            self.status.setText('Selecciona el termo para calibrar el frente y continuar.');return
        self.stage=stage;self.pages.setCurrentIndex(stage)
        self.back_button.setVisible(stage==1);self.next_button.setVisible(stage==0);self.apply_button.setVisible(stage==1)
        if stage==1:self.live=False
        self.instructions.setText('Etapa 1 de 2: ajusta el giro del LiDAR respecto al AGV. El escaneo se actualiza en vivo; verifica delante e izquierda antes de continuar.' if stage==0 and self.live else 'Etapa 1 de 2: CAPTURA CONGELADA. Ajusta el giro y verifica delante e izquierda, o reanuda la vista en vivo.' if stage==0 else 'Etapa 2 de 2: ajusta la zona que se ignorará. Verde: usar; naranja: excluir. Confirma la captura y guarda.')
        self.redraw();self.update_apply()

    def fit_points(self):
        # Explicit limits include every valid return, regardless of the guide circle.
        dx=self.points-np.asarray(self.mount[:2])
        delta=math.radians(0 if self.candidate is None else self.candidate-self.mount[2])
        rotation=np.array([[math.cos(delta),-math.sin(delta)],[math.sin(delta),math.cos(delta)]])
        drawn=dx@rotation.T+np.asarray(self.mount[:2])
        all_points=np.vstack((drawn,np.asarray(self.mount[:2]),np.array([[0.,0.]])))
        low=all_points.min(axis=0);high=all_points.max(axis=0)
        span=max(float(np.max(high-low)),.6);center=(low+high)/2
        self.plot.setRange(xRange=(center[0]-span*.55,center[0]+span*.55),yRange=(center[1]-span*.55,center[1]+span*.55),padding=0)

    def keep_orientation(self):
        self.candidate=self.current_angle;self.selected=None;self.live=False;self.selection.setData([],[])
        self.set_stage(1);self.fit_points()

    def preview_opposite_mount(self, saved_center, saved_width):
        def opposite(angle):
            return angle - 180 if angle > 0 else angle + 180

        angle = opposite(self.current_angle)
        self.angle_spin.setValue(angle)
        self.adjust_angle(angle)
        self.center.setValue(round(opposite(saved_center)))
        self.width.setValue(round(saved_width))
        self.still.setChecked(False)
        self.set_stage(1)
        self.fit_points()

    def adjust_angle(self,value):
        self.angle_slider.blockSignals(True);self.angle_slider.setValue(round(value*10));self.angle_slider.blockSignals(False)
        self.candidate=value;self.selected=None;self.selection.setData([],[])
        self.redraw();self.update_apply()

    def zoom(self,factor):
        self.plot.getViewBox().scaleBy((factor,factor))

    def replace_points(self,points):
        self.points=np.asarray(points,dtype=float).reshape(-1,2).copy()
        self.candidate=None;self.selected=None;self.live=True;self.selection.setData([],[]);self.still.setChecked(False)
        self.set_stage(0);self.fit_points()

    def update_live_points(self,points):
        if self.stage!=0 or not self.live:return
        self.points=np.asarray(points,dtype=float).reshape(-1,2).copy()
        self.redraw()

    def sector_control(self,form,label,minimum,maximum,value):
        container=QtWidgets.QWidget();row=QtWidgets.QHBoxLayout(container)
        row.setContentsMargins(0,0,0,0)
        slider=QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal);slider.setRange(minimum,maximum)
        spin=QtWidgets.QSpinBox();spin.setRange(minimum,maximum);spin.setSuffix('°');spin.setValue(round(value))
        slider.setValue(spin.value());slider.valueChanged.connect(spin.setValue);spin.valueChanged.connect(slider.setValue)
        row.addWidget(slider,1);row.addWidget(spin);form.addRow(label,container)
        return spin,slider

    def redraw(self):
        dx=self.points-np.asarray(self.mount[:2])
        delta=math.radians(0 if self.candidate is None else self.candidate-self.mount[2])
        rotation=np.array([[math.cos(delta),-math.sin(delta)],[math.sin(delta),math.cos(delta)]])
        drawn=dx@rotation.T+np.asarray(self.mount[:2])
        excluded=(exclusion_mask(drawn,self.mount[:2],self.body_margin,self.center.value(),self.width.value())
                  if self.stage==1 else np.zeros(len(drawn),dtype=bool))
        self.scan.setData(drawn[~excluded,0],drawn[~excluded,1])
        self.excluded_scan.setData(drawn[excluded,0],drawn[excluded,1])
        radius=max(.5,min(4.,float(np.percentile(np.linalg.norm(dx,axis=1),90)))) if len(dx) else 1.
        theta=np.linspace(-math.pi,math.pi,181);origin=np.asarray(self.mount[:2])
        self.circle.setData(origin[0]+radius*np.cos(theta),origin[1]+radius*np.sin(theta))
        if self.stage==1 and self.width.value():
            arc=np.radians(np.linspace(self.center.value()-self.width.value()/2,self.center.value()+self.width.value()/2,181))
            self.mask_outline.setData(np.r_[origin[0],origin[0]+radius*np.cos(arc),origin[0]],np.r_[origin[1],origin[1]+radius*np.sin(arc),origin[1]])
        else:self.mask_outline.setData([],[])
        if self.candidate is None:
            # Before choosing the front, raw sensor axes are not robot axes.
            self.body.setData([],[])
            self.plot.setTitle('Vista sin orientar · selecciona el objeto situado delante')
        else:
            self.body.setData([-.115,.115,.115,-.115,-.115],[-.115,-.115,.115,.115,-.115])
            self.plot.setTitle('Vista relativa al AGV · frente a la derecha · izquierda arriba')
        self.sensor.setData([self.mount[0]],[self.mount[1]])
        if self.selected is not None:
            selected=(self.points[self.selected]-np.asarray(self.mount[:2]))@rotation.T+np.asarray(self.mount[:2])
            self.selection.setData([selected[0]],[selected[1]])
        front_excluded=self.stage==1 and self.width.value()>0 and abs((0-self.center.value()+180)%360-180)<=self.width.value()/2
        message=('EN VIVO · ' if self.stage==0 and self.live else 'CONGELADO · ')+f'{np.count_nonzero(~excluded)} retornos visibles · {np.count_nonzero(excluded)} excluidos.'
        if front_excluded:message+=' Atención: el sector excluido incluye el frente.'
        if self.candidate is None:message+=' Selecciona el objeto situado delante.'
        else:message+=f' Giro propuesto: {self.candidate:+.1f}°.'
        self.status.setText(message)

    def choose_point(self,x,y):
        if self.stage!=0 or not len(self.points):return
        # After a selection the drawing has been rotated, so map clicks back
        # into the original snapshot before selecting another return.
        delta=math.radians(0 if self.candidate is None else self.candidate-self.mount[2])
        c,s=math.cos(delta),math.sin(delta)
        p=np.array([x,y])-np.asarray(self.mount[:2])
        original=np.array([c*p[0]+s*p[1],-s*p[0]+c*p[1]])+np.asarray(self.mount[:2])
        distances=np.linalg.norm(self.points-original,axis=1);index=int(np.argmin(distances))
        if distances[index]>.15:
            self.status.setText('No hay un retorno cerca del clic. Selecciona los puntos del termo.');return
        dx,dy=self.points[index]-np.asarray(self.mount[:2])
        if math.hypot(dx,dy)<self.mount[3]:
            self.status.setText('Elige un retorno válido del objeto delante.');return
        bearing=math.degrees(math.atan2(dy,dx))
        self.candidate=(self.mount[2]-bearing+180)%360-180;self.selected=index
        self.angle_spin.blockSignals(True);self.angle_spin.setValue(self.candidate);self.angle_spin.blockSignals(False)
        self.angle_slider.blockSignals(True);self.angle_slider.setValue(round(self.candidate*10));self.angle_slider.blockSignals(False)
        self.live=False
        self.status.setText(f'Giro propuesto: {self.candidate:+.1f}°. Verifica que el termo quede delante y confirma que nada se movió.')
        self.set_stage(0);self.fit_points()

    def update_apply(self,*_args):
        self.apply_button.setEnabled(self.candidate is not None and self.still.isChecked() and self.width.value()<360)
        reasons=[]
        if self.candidate is None:reasons.append('ajusta el giro o selecciona el frente')
        if not self.still.isChecked():reasons.append('confirma que el AGV estaba quieto')
        if self.width.value()>=360:reasons.append('reduce el ancho: 360° excluye todos los puntos')
        self.requirements.setText('Para guardar: '+ '; '.join(reasons)+'.' if reasons else 'Listo para guardar la configuración y crear un mapa nuevo.')
