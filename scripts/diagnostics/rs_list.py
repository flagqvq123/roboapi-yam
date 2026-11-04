import pyrealsense2 as rs
ctx = rs.context()
for d in ctx.query_devices():
    sn = d.get_info(rs.camera_info.serial_number)
    name = d.get_info(rs.camera_info.name)
    print(f"== {name} {sn} ==")
    for s in d.sensors:
        try:
            sname = s.get_info(rs.camera_info.name)
        except Exception as e:
            sname = f"<{e}>"
        print("  sensor:", sname)
        kinds = {}
        for p in s.get_stream_profiles():
            try:
                v = p.video_stream_profile()
                key = (str(v.stream_type()), str(v.format()), v.width(), v.height(), v.fps())
                kinds[key] = True
            except Exception:
                pass
        for k in sorted(kinds)[:20]:
            print("     ", k)
