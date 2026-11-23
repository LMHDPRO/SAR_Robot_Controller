import sys
import time
import serial
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                               QHBoxLayout, QLabel, QSlider, QPushButton, QLineEdit,
                               QGroupBox, QMessageBox)
from PySide6.QtCore import Qt, QTimer

class CalibradorRobot(QMainWindow):
    def __init__(self):
        super().__init__()
        self.conexion = None
        self.initUI()
        
        # Timer para enviar datos por Serial a 10Hz (cada 100ms)
        # Esto permite que muevas el slider y el robot responda suavemente en tiempo real
        self.timer = QTimer()
        self.timer.timeout.connect(self.enviar_datos)
        
    def initUI(self):
        self.setWindowTitle("Calibración Directa de Servomotores")
        self.resize(400, 450)
        
        widget_central = QWidget()
        layout_principal = QVBoxLayout()
        
        # --- Panel de Conexión ---
        grupo_conexion = QGroupBox("Puerto Serial del ESP32")
        layout_conexion = QHBoxLayout()
        self.txt_puerto = QLineEdit("COM3")
        self.btn_conectar = QPushButton("Conectar")
        self.btn_conectar.clicked.connect(self.conectar_serial)
        layout_conexion.addWidget(self.txt_puerto)
        layout_conexion.addWidget(self.btn_conectar)
        grupo_conexion.setLayout(layout_conexion)
        layout_principal.addWidget(grupo_conexion)
        
        # --- Panel de Sliders (Hardware Puro) ---
        self.grupo_sliders = QGroupBox("Control Directo (0° - 180°)")
        layout_sliders = QVBoxLayout()
        self.sliders = {}
        
        # Definimos los motores y sus posiciones iniciales de seguridad
        motores = [
            ("Base", 90),
            ("Hombro", 90),
            ("Codo", 90),
            ("Muñeca Giro", 115),
            ("Muñeca Elevación", 90),
            ("Garra", 110)
        ]
        
        for nombre, inicio in motores:
            row = QHBoxLayout()
            lbl = QLabel(f"{nombre}: {inicio}°")
            lbl.setFixedWidth(130)
            sl = QSlider(Qt.Horizontal)
            sl.setRange(0, 180)
            sl.setValue(inicio)
            
            # Actualiza el texto en la interfaz inmediatamente
            sl.valueChanged.connect(lambda val, l=lbl, n=nombre: l.setText(f"{n}: {val}°"))
            
            row.addWidget(lbl)
            row.addWidget(sl)
            layout_sliders.addLayout(row)
            self.sliders[nombre] = sl
            
        self.grupo_sliders.setLayout(layout_sliders)
        self.grupo_sliders.setEnabled(False) # Deshabilitado hasta conectar
        layout_principal.addWidget(self.grupo_sliders)
        
        widget_central.setLayout(layout_principal)
        self.setCentralWidget(widget_central)
        
    def conectar_serial(self):
        if self.conexion and self.conexion.is_open:
            # Si ya está conectado, desconectamos
            self.timer.stop()
            self.conexion.close()
            self.btn_conectar.setText("Conectar")
            self.grupo_sliders.setEnabled(False)
            self.txt_puerto.setEnabled(True)
        else:
            # Intentar conectar
            puerto = self.txt_puerto.text()
            try:
                self.conexion = serial.Serial(puerto, 115200, timeout=1)
                
                # Evitar Auto-Reset del ESP32
                self.conexion.setDTR(False)
                self.conexion.setRTS(False)
                
                self.btn_conectar.setText("Iniciando ESP32...")
                self.btn_conectar.setEnabled(False)
                self.txt_puerto.setEnabled(False)
                QApplication.processEvents() # Evita que la ventana se congele
                
                time.sleep(2) # Dar tiempo al ESP32 para despertar
                
                self.conexion.reset_input_buffer()
                self.conexion.reset_output_buffer()
                
                self.btn_conectar.setText("Desconectar")
                self.btn_conectar.setEnabled(True)
                self.grupo_sliders.setEnabled(True)
                
                # Iniciar el envío continuo de datos
                self.timer.start(100) 
                
            except serial.SerialException as e:
                QMessageBox.critical(self, "Error de Conexión", f"No se pudo abrir el puerto {puerto}.\n\nAsegúrate de que el Monitor Serie de Arduino esté cerrado.")
                self.btn_conectar.setText("Conectar")
                self.btn_conectar.setEnabled(True)
                self.txt_puerto.setEnabled(True)
                
    def enviar_datos(self):
        """Lee los sliders y envía la cadena formateada al ESP32"""
        if self.conexion and self.conexion.is_open:
            ang_base = self.sliders["Base"].value()
            ang_hombro = self.sliders["Hombro"].value()
            ang_codo = self.sliders["Codo"].value()
            ang_m_giro = self.sliders["Muñeca Giro"].value()
            ang_m_elev = self.sliders["Muñeca Elevación"].value()
            ang_garra = self.sliders["Garra"].value()
            
            # Tiempo de movimiento rápido (200ms) para que responda fluido a tus dedos
            tiempo = 200 
            
            # Formato exacto que espera tu ESP32
            cadena = f"{ang_base:.1f},{ang_hombro:.1f},{ang_codo:.1f},{ang_m_giro:.1f},{ang_m_elev:.1f},{ang_garra:.1f},{tiempo}\n"
            
            try:
                self.conexion.write(cadena.encode('ascii'))
            except serial.SerialException:
                # Si el cable se desconecta por accidente
                self.conectar_serial()
            
    def closeEvent(self, event):
        """Cierra el puerto serial de forma segura al salir"""
        if self.conexion and self.conexion.is_open:
            self.timer.stop()
            self.conexion.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ventana = CalibradorRobot()
    ventana.show()
    sys.exit(app.exec())
    