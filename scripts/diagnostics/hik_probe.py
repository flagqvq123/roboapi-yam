#!/usr/bin/env python3
"""Grab one frame from every Hikrobot camera so the operator can identify each view.

Usage: python scripts/hik_probe.py [--out DIR]
"""
import argparse
import ctypes
import os
import pathlib
import sys

import cv2
import numpy as np

SDK_ROOT = pathlib.Path(os.environ.get("MVCAM_SDK_PATH", "/opt/MVS"))


def load_sdk():
    wrapper = SDK_ROOT / "Samples/64/Python/MvImport"
    if not wrapper.is_dir():
        raise SystemExit("MVS Python SDK missing: " + str(wrapper))
    os.environ.setdefault("MVCAM_COMMON_RUNENV", str(SDK_ROOT / "lib"))
    if str(wrapper) not in sys.path:
        sys.path.insert(0, str(wrapper))
    import MvCameraControl_class as sdk
    ret = sdk.MvCamera.MV_CC_Initialize()
    if ret != 0:
        print("Initialize ret", hex(ret))
    return sdk


def serial_of(info):
    """The SDK stores the serial as a fixed c_ubyte array."""
    for attr in ("stUsb3VInfo", "stGigEInfo", "stUsb3VInfo"):
        try:
            raw = getattr(info.SpecialInfo, attr).chSerialNumber
        except Exception:
            continue
        try:
            return bytes(raw).decode("utf-8", "ignore").strip(chr(0)).strip()
        except Exception:
            return "unknown"
    return "unknown"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="/home/ps/roboapi/logs/hik")
    args = parser.parse_args()
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    sdk = load_sdk()
    devices = sdk.MV_CC_DEVICE_INFO_LIST()
    ret = sdk.MvCamera.MV_CC_EnumDevices(sdk.MV_GIGE_DEVICE | sdk.MV_USB_DEVICE, devices)
    if ret != 0:
        raise SystemExit("EnumDevices failed: " + hex(ret))
    print("devices:", devices.nDeviceNum)
    for i in range(devices.nDeviceNum):
        info = ctypes.cast(devices.pDeviceInfo[i], ctypes.POINTER(sdk.MV_CC_DEVICE_INFO)).contents
        serial = serial_of(info).strip()
        cam = sdk.MvCamera()
        created = opened = grabbing = False
        try:
            ret = cam.MV_CC_CreateHandle(info)
            if ret != 0:
                print(f"[{i}] create failed {hex(ret)}"); continue
            created = True
            ret = cam.MV_CC_OpenDevice(sdk.MV_ACCESS_Exclusive, 0)
            if ret != 0:
                print(f"[{i}] {serial}: open failed {hex(ret)}"); continue
            opened = True
            cam.MV_CC_SetEnumValue("TriggerMode", 0)
            cam.MV_CC_SetEnumValue("PixelFormat", sdk.PixelType_Gvsp_BayerRG8)
            ret = cam.MV_CC_StartGrabbing()
            if ret != 0:
                print(f"[{i}] {serial}: start failed {hex(ret)}"); continue
            grabbing = True
            frame = sdk.MV_FRAME_OUT()
            code = cam.MV_CC_GetImageBuffer(frame, 2000)
            if (code & 0xffffffff) != 0:
                print(f"[{i}] {serial}: no frame {hex(code)}"); continue
            try:
                meta = frame.stFrameInfo
                width, height = int(meta.nWidth), int(meta.nHeight)
                raw = np.ctypeslib.as_array(
                    ctypes.cast(frame.pBufAddr, ctypes.POINTER(ctypes.c_ubyte)),
                    shape=(width * height,)).copy().reshape(height, width)
                bgr = cv2.cvtColor(raw, cv2.COLOR_BayerRG2BGR)
                name = f"hik_{i}_{serial or 'noserial'}.png"
                cv2.imwrite(str(out / name), bgr)
                print(f"[{i}] {serial}: {width}x{height} -> {out / name}")
            finally:
                cam.MV_CC_FreeImageBuffer(frame)
        finally:
            if grabbing:
                cam.MV_CC_StopGrabbing()
            if opened:
                cam.MV_CC_CloseDevice()
            if created:
                cam.MV_CC_DestroyHandle()
    sdk.MvCamera.MV_CC_Finalize()


if __name__ == "__main__":
    main()
