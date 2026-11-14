"""Isolated ESP32 MJPEG capture + ArUco. stdout is newline JSON, never motion commands."""
import base64
import json
import sys
import time
import cv2
import numpy as np


def load_calibration(path):
    if not path:return None
    with np.load(path,allow_pickle=False) as data:
        k=np.asarray(data['camera_matrix'],dtype=float)
        d=np.asarray(data['dist_coeffs'],dtype=float).reshape(-1)
        size=np.asarray(data['image_size'],dtype=float)
    if (k.shape!=(3,3) or d.size not in (4,5,8,12,14) or size.shape!=(2,)
        or not all(np.all(np.isfinite(x)) for x in (k,d,size))
        or np.any(size<=0) or k[0,0]<=0 or k[1,1]<=0):
        raise ValueError('Calibración inválida: camera_matrix, dist_coeffs, image_size=[ancho,alto]')
    return k,d,size


def detect(frame,detector,marker_mm,calibration):
    corners,ids,_=detector.detectMarkers(frame)
    rows=[]
    if ids is None:return rows
    cv2.aruco.drawDetectedMarkers(frame,corners,ids)
    for c,identifier in zip(corners,ids.reshape(-1)):
        row={'id':int(identifier),'center_px':np.mean(c.reshape(4,2),axis=0).tolist()}
        if calibration is not None:
            k,d,size=calibration
            h,w=frame.shape[:2]
            # Resolution changes may scale uniformly; cropping requires new calibration.
            if abs((w/h)/(size[0]/size[1])-1)>.01:
                row['error']='Relación de aspecto distinta de la calibración'
            else:
                k=k.copy();k[0,:]*=w/size[0];k[1,:]*=h/size[1]
                half=marker_mm/2
                obj=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]],np.float32)
                ok,r,t=cv2.solvePnP(obj,c.reshape(4,2),k,d,flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if ok and np.all(np.isfinite(t)) and t[2,0]>0:
                    projected,_=cv2.projectPoints(obj,r,t,k,d)
                    error=float(np.sqrt(np.mean(np.sum((projected.reshape(4,2)-c.reshape(4,2))**2,axis=1))))
                    row['reprojection_px']=error
                    if error<=3.0:
                        row.update(xyz_camera_mm=t.reshape(3).tolist(),distance_mm=float(np.linalg.norm(t)))
                        cv2.drawFrameAxes(frame,k,d,r,t,marker_mm*.5)
                    else:row['error']='Pose descartada: reproyección >3 px'
        rows.append(row)
    return rows


def emit(value):
    print(json.dumps(value,separators=(',',':')),flush=True)


def run(config):
    calibration=load_calibration(config.get('calibration',''))
    marker_mm=float(config['marker_mm'])
    if not np.isfinite(marker_mm) or marker_mm<=0:raise ValueError('Tamaño de marcador inválido')
    dictionary=cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,config['dictionary']))
    detector=cv2.aruco.ArucoDetector(dictionary,cv2.aruco.DetectorParameters())
    source=config['url'].strip()
    if not source.startswith(('http://','https://','rtsp://')):raise ValueError('Introducir URL http(s) o rtsp de la cámara ESP32')
    cap=cv2.VideoCapture(source,cv2.CAP_FFMPEG,[cv2.CAP_PROP_OPEN_TIMEOUT_MSEC,3000,cv2.CAP_PROP_READ_TIMEOUT_MSEC,2000])
    if not cap.isOpened():raise RuntimeError('No se pudo abrir el stream de la ESP32-S3')
    cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
    try:
        while True:
            start=time.monotonic()
            ok,frame=cap.read()
            if not ok:raise RuntimeError('Stream perdido; reconectar cámara')
            rows=detect(frame,detector,marker_mm,calibration)
            h,w=frame.shape[:2]
            preview=cv2.resize(frame,(min(640,w),round(h*min(640,w)/w)))
            ok,jpeg=cv2.imencode('.jpg',preview,[cv2.IMWRITE_JPEG_QUALITY,75])
            if ok:emit({'jpeg':base64.b64encode(jpeg).decode(),'detections':rows,'metric':calibration is not None})
            time.sleep(max(0,.1-(time.monotonic()-start)))
    finally:cap.release()


if __name__=='__main__':
    try:run(json.loads(sys.argv[1]))
    except Exception as exc:
        emit({'error':str(exc)});sys.exit(1)
