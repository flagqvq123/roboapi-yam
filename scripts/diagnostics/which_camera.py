#!/usr/bin/env python3
"""Identify which camera is mounted on a moving arm: capture, wait, capture, rank by change."""
import ctypes
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

SDK_ROOT = Path(os.environ.get("MVCAM_SDK_PATH", "/opt/MVS"))


def real_sense_serials():
    import pyrealsense2 as rs
    return [d.get_info(rs.camera_info.serial_number) for d in rs.context().query_devices()]


def grab_rs(serial):
    import pyrealsense2 as rs
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipe.start(cfg)
    try:
        img = np.asanyarray(pipe.wait_for_frames(3000).get_color_frame().get_data()).copy()
    finally:
        try:
            pipe.stop()
        except Exception:
            pass
    return img


def load_sdk():
    wrap = SDK_ROOT / "Samples/64/Python/MvImport"
    os.environ.setdefault("MVCAM_COMMON_RUNENV", str(SDK_ROOT / "lib"))
    sys.path.insert(0, str(wrap))
    import MvCameraControl_class as sdk
    sdk.MvCamera.MV_CC_Initialize()
    return sdk


def hik_devices(sdk):
    lst = sdk.MV_CC_DEVICE_INFO_LIST()
    sdk.MvCamera.MV_CC_EnumDevices(sdk.MV_GIGE_DEVICE | sdk.MV_USB_DEVICE, lst)
    out = []
    for i in range(lst.nDeviceNum):
        info = ctypes.cast(lst.pDeviceInfo[i], ctypes.POINTER(sdk.MV_CC_DEVICE_INFO)).contents
        sn = bytes(info.SpecialInfo.stUsb3VInfo.chSerialNumber).decode("utf-8", "ignore").strip(chr(0))
        out.append((sn, info))
    return out


def grab_hik(sdk, info):
    cam = sdk.MvCamera()
    created = opened = grabbing = False
    try:
        if cam.MV_CC_CreateHandle(info) != 0:
            raise RuntimeError("create")
        created = True
        if cam.MV_CC_OpenDevice(sdk.MV_ACCESS_Exclusive, 0) != 0:
            raise RuntimeError("busy")
        opened = True
        cam.MV_CC_SetEnumValue("TriggerMode", 0)
        cam.MV_CC_SetEnumValue("PixelFormat", sdk.PixelType_Gvsp_BayerRG8)
        if cam.MV_CC_StartGrabbing() != 0:
            raise RuntimeError("start")
        grabbing = True
        frame = sdk.MV_FRAME_OUT()
        if (cam.MV_CC_GetImageBuffer(frame, 1500) & 0xffffffff) != 0:
            raise RuntimeError("no frame")
        try:
            meta = frame.stFrameInfo
            w, h = int(meta.nWidth), int(meta.nHeight)
            raw = np.ctypeslib.as_array(
                ctypes.cast(frame.pBufAddr, ctypes.POINTER(ctypes.c_ubyte)),
                shape=(w * h,)).copy().reshape(h, w)
            return cv2.cvtColor(raw, cv2.COLOR_BayerRG2BGR)
        finally:
            cam.MV_CC_FreeImageBuffer(frame)
    finally:
        if grabbing:
            cam.MV_CC_StopGrabbing()
        if opened:
            cam.MV_CC_CloseDevice()
        if created:
            cam.MV_CC_DestroyHandle()


def capture_all():
    shots = {}
    try:
        for serial in real_sense_serials():
            try:
                shots["RS " + serial] = grab_rs(serial)
            except Exception as exc:
                print("  RS", serial, "unavailable:", str(exc)[:70])
    except Exception as exc:
        print("  realsense unavailable:", str(exc)[:70])
    try:
        sdk = load_sdk()
        for serial, info in hik_devices(sdk):
            try:
                shots["HIK " + serial] = grab_hik(sdk, info)
            except Exception as exc:
                print("  HIK", serial, "unavailable:", str(exc)[:70])
        sdk.MvCamera.MV_CC_Finalize()
    except Exception as exc:
        print("  MVS unavailable:", str(exc)[:70])
    return shots


def main():
    print("round 1 ...")
    first = capture_all()
    print("captured:", len(first))
    print(chr(10) + ">>> 现在请用手缓慢移动【左臂】，保持 5 秒 <<<" + chr(10), flush=True)
    time.sleep(5)
    print("round 2 ...")
    second = capture_all()
    scores = []
    for name, img_a in first.items():
        img_b = second.get(name)
        if img_b is None or img_b.shape != img_a.shape:
            continue
        a = cv2.cvtColor(img_a, cv2.COLOR_BGR2GRAY).astype(np.float32)
        b = cv2.cvtColor(img_b, cv2.COLOR_BGR2GRAY).astype(np.float32)
        scores.append((float(np.mean(np.abs(a - b))), name))
    scores.sort(reverse=True)
    print(chr(10) + "相机画面变化排名（越大 = 越可能装在移动的那条臂上）：")
    for score, name in scores:
        print("  %8.2f  %s" % (score, name))


if __name__ == "__main__":
    main()
