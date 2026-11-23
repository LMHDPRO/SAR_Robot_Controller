import sys
import math
import time
import serial
import pybullet as p
import pybullet_data
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                               QHBoxLayout, QLabel, QDoubleSpinBox, 
                               QComboBox, QGroupBox)
from PySide6.QtCore import Qt, QThread, Signal, Slot

# ==========================================
# 1. EL "WORKER" (PyBullet y Serial)
# ==========================================
class SimulacionRobot(QThread):
    angulos_actualizados = Signal(list)
    estado_conexion = Signal(str)

    def __init__(self, ruta_urdf, puerto_serial="COM3"):
        super().__init__()
        self.ruta_urdf = ruta_urdf
        self.puerto_serial = puerto_serial
        self.corriendo = True
        
        # Variables compartidas con la UI
        self.pos_objetivo = [0.17, 0, 0.30]
        self.pitch_deg = 0.0
        
        # === CALIBRACIÓN FÍSICA ===
        # Índices: [0: Base, 1: Hombro, 2: Codo, 3: Muñeca Elevación]
        self.servo_offset = [12.0, 49.0, 75.0, 106.0] 
        self.servo_dir    = [ 1.0,  1.0,  1.0,  1.0]
        
        # Variables fijas de inicio
        self.muneca_giro = 115.0
        self.gripper = 110.0
        self.tiempo = 1000

    def inicializar_serial(self):
        try:
            conexion = serial.Serial(self.puerto_serial, 115200, timeout=1)
            conexion.setDTR(False)
            conexion.setRTS(False)
            print("Esperando 2 segundos a que el ESP32 inicie...")
            time.sleep(2)
            conexion.reset_input_buffer()
            conexion.reset_output_buffer()
            
            self.estado_conexion.emit(f"Serial CONECTADO ({self.puerto_serial})")
            return conexion
        except serial.SerialException:
            self.estado_conexion.emit("Serial DESCONECTADO (Simulación pura)")
            return None

    def enviar_datos_serial(self, conexion, ang_base, ang_hombro, ang_codo, ang_muneca_elev):
        cadena_serial = f"{ang_base:.1f},{ang_hombro:.1f},{ang_codo:.1f},{self.muneca_giro:.1f},{ang_muneca_elev:.1f},{self.gripper},{self.tiempo}\n"
        print(f"ENVIANDO -> {cadena_serial.strip()}")
        if conexion and conexion.is_open:
            conexion.write(cadena_serial.encode('ascii'))

    def run(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.resetDebugVisualizerCamera(0.8, -45, -30, [0, 0, 0.1])
        
        robot_id = p.loadURDF(self.ruta_urdf, basePosition=[0, 0, 0], useFixedBase=True)
        
        actuadores = [i for i in range(p.getNumJoints(robot_id)) if p.getJointInfo(robot_id, i)[2] == p.JOINT_REVOLUTE]
        tcp_idx = next((i for i in range(p.getNumJoints(robot_id)) if p.getJointInfo(robot_id, i)[12].decode('utf-8') == "tcp_punta_garra"), -1)
        
        conexion_serial = self.inicializar_serial()
        ultimo_envio = time.time()
        
        esfera_visual = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.015, rgbaColor=[1, 0.5, 0, 1])
        marcador_obj = p.createMultiBody(baseMass=0, baseVisualShapeIndex=esfera_visual, basePosition=[0,0,0])

        try:
            while self.corriendo:
                x, y, z = self.pos_objetivo
                pitch_rad = math.radians(self.pitch_deg)
                
                p.resetBasePositionAndOrientation(marcador_obj, [x,y,z], [0,0,0,1])
                
                yaw_base = math.atan2(y, x)
                cuat_objetivo = p.getQuaternionFromEuler([0, pitch_rad, yaw_base])
                
                angulos_ik = p.calculateInverseKinematics(
                    bodyUniqueId=robot_id,
                    endEffectorLinkIndex=tcp_idx,
                    targetPosition=[x, y, z],
                    targetOrientation=cuat_objetivo,
                    maxNumIterations=100,
                    residualThreshold=1e-5
                )
                
                for idx, motor_id in enumerate(actuadores):
                    p.setJointMotorControl2(robot_id, motor_id, p.POSITION_CONTROL, targetPosition=angulos_ik[idx], force=1000.0)
                
                ang_deg = [math.degrees(a) for a in angulos_ik]
                
                # === APLICAR MAPEO Y LÍMITES A LOS MOTORES FÍSICOS ===
                ang_fisicos = []
                for i in range(4):
                    # Ecuación: (Simulación * Dirección) + Offset
                    val_fisico = (ang_deg[i] * self.servo_dir[i]) + self.servo_offset[i]
                    # Limitar estrictamente entre 0 y 180 por seguridad mecánica
                    val_fisico = max(0.0, min(180.0, val_fisico))
                    ang_fisicos.append(val_fisico)
                
                # Emitir a la Interfaz Gráfica los valores reales FÍSICOS
                self.angulos_actualizados.emit([ang_fisicos[0], ang_fisicos[1], ang_fisicos[2], 
                                                self.muneca_giro, ang_fisicos[3], self.gripper, self.tiempo])
                
                if (time.time() - ultimo_envio) >= 0.1:
                    self.enviar_datos_serial(conexion_serial, ang_fisicos[0], ang_fisicos[1], 
                                             ang_fisicos[2], ang_fisicos[3])
                    ultimo_envio = time.time()
                
                p.stepSimulation()
                time.sleep(1./240.)
                
        except p.error:
            pass 
        finally:
            if conexion_serial and conexion_serial.is_open:
                conexion_serial.close()
            p.disconnect()

    def detener(self):
        self.corriendo = False
        self.wait()

# ==========================================
# 2. LA INTERFAZ GRÁFICA (PySide6)
# ==========================================
class PanelControlUI(QMainWindow):
    def __init__(self, trabajador_simulacion):
        super().__init__()
        self.sim = trabajador_simulacion
        self.initUI()
        
        self.sim.angulos_actualizados.connect(self.actualizar_textos_angulos)
        self.sim.estado_conexion.connect(self.lbl_estado_serial.setText)

    def initUI(self):
        self.setWindowTitle("Controlador Kinemático de Brazo Robótico")
        self.resize(400, 500)
        
        widget_central = QWidget()
        layout_principal = QVBoxLayout()
        
        grupo_modos = QGroupBox("Modo de Operación")
        layout_modos = QVBoxLayout()
        self.combo_modos = QComboBox()
        self.combo_modos.addItems([
            "Manual XYZ + Pitch",
            "Control Remoto (Pendiente)",
            "Visión Artificial (Pendiente)",
            "Secuencia / Evasor (Pendiente)",
            "Manual Articular (Pendiente)"
        ])
        layout_modos.addWidget(self.combo_modos)
        grupo_modos.setLayout(layout_modos)
        layout_principal.addWidget(grupo_modos)
        
        grupo_manual = QGroupBox("Control de Coordenadas (Modo Manual)")
        layout_manual = QVBoxLayout()
        self.spins = {} 
        
        controles = [
            ("X (m)", -0.5, 0.5, -0.2, 0.01),
            ("Y (m)", -0.5, 0.5, 0.1, 0.01),
            ("Z (m)", 0.0, 0.6, 0.15, 0.01),
            ("Pitch (°)", -90, 90, 0, 1.0)
        ]
        
        for nombre, v_min, v_max, v_ini, step in controles:
            row = QHBoxLayout()
            row.addWidget(QLabel(nombre))
            
            spin = QDoubleSpinBox()
            spin.setRange(v_min, v_max)
            spin.setValue(v_ini)
            spin.setSingleStep(step)
            spin.valueChanged.connect(self.enviar_coordenadas_al_worker)
            
            row.addWidget(spin)
            layout_manual.addLayout(row)
            self.spins[nombre] = spin
            
        grupo_manual.setLayout(layout_manual)
        layout_principal.addWidget(grupo_manual)
        
        grupo_salida = QGroupBox("Salida de Datos (Físicos)")
        self.layout_salida = QVBoxLayout()
        self.lbl_angulos = QLabel("Esperando datos...")
        self.layout_salida.addWidget(self.lbl_angulos)
        grupo_salida.setLayout(self.layout_salida)
        layout_principal.addWidget(grupo_salida)
        
        self.lbl_estado_serial = QLabel("Iniciando conexión...")
        self.lbl_estado_serial.setStyleSheet("font-weight: bold; color: blue;")
        layout_principal.addWidget(self.lbl_estado_serial)
        
        widget_central.setLayout(layout_principal)
        self.setCentralWidget(widget_central)

    @Slot()
    def enviar_coordenadas_al_worker(self):
        self.sim.pos_objetivo = [
            self.spins["X (m)"].value(),
            self.spins["Y (m)"].value(),
            self.spins["Z (m)"].value()
        ]
        self.sim.pitch_deg = self.spins["Pitch (°)"].value()

    @Slot(list)
    def actualizar_textos_angulos(self, datos):
        texto = f"Base: {datos[0]:.1f}°\n"
        texto += f"Hombro: {datos[1]:.1f}°\n"
        texto += f"Codo: {datos[2]:.1f}°\n"
        texto += f"Muñeca Giro: {datos[3]:.1f}° (Fijo)\n"
        texto += f"Muñeca Elev: {datos[4]:.1f}°\n"
        texto += f"Gripper: {datos[5]}\n"
        texto += f"Tiempo: {datos[6]} ms"
        self.lbl_angulos.setText(texto)

    def closeEvent(self, event):
        self.sim.detener()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    # Revisa que la ruta del URDF sea la correcta (tienes dos archivos, brazo_limpio.urdf y brazo_limpio - copia.urdf)
    hilo_simulacion = SimulacionRobot(ruta_urdf="urdf/brazo_limpio - copia.urdf", puerto_serial="COM3") 
    hilo_simulacion.start()
    ventana = PanelControlUI(hilo_simulacion)
    ventana.show()
    sys.exit(app.exec())