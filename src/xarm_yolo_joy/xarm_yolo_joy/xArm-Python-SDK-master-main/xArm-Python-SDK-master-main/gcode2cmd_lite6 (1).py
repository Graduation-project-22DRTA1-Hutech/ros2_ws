if __name__ == "__main__":
    path = input("Enter path of G-code file:  ")
    Z_offset = 10
    try:
        user_cordinate = input("Enter init coordinate in form x,y,z:  ").split(',')
        user_cordinate = list(map(float, user_cordinate))
    except ValueError:
        print("Invalid input format. Please enter coordinates in the form x,y,z.")
        exit()

    try:
        with open(path, 'r') as file:
            data = file.readlines()
    except FileNotFoundError:
        print(f"File {path} not found.")
        exit()

    with open('converted_output_lite6.txt', 'w') as output_file:
        # Servo On & home position
        output_file.write("arm.set_position(x=261., y=0, z=258.3, roll=180, pitch=0, yaw=0, speed=50, wait=True)\n")
        output_file.write("time.sleep(0.1)\n")
        # Initial pose
        output_file.write(f"arm.set_position(x={user_cordinate[0]}, y={user_cordinate[1]}, z={user_cordinate[2] + Z_offset}, roll=180, pitch=0, yaw=0, speed=50, wait=True)\n")
        output_file.write("time.sleep(0.1)\n")
        output_file.write(f"arm.set_position(x={user_cordinate[0]}, y={user_cordinate[1]}, z={user_cordinate[2]}, roll=180, pitch=0, yaw=0, speed=50, wait=True)\n")
        output_file.write("time.sleep(0.1)\n")
        output_file.write(f"arm.set_position(x={user_cordinate[0]}, y={user_cordinate[1]}, z={user_cordinate[2] + Z_offset}, roll=180, pitch=0, yaw=0, speed=50, wait=True)\n")
        output_file.write("time.sleep(0.1)\n")
        last_x = None
        last_y = None

        for line_index, line in enumerate(data, start=1):
            line = line.strip()
            if not (line.startswith("G00") or line.startswith("G01")):
                continue

            parts = line.split()
            x_val = y_val = None
            for p in parts:
                if p.startswith("X"):
                    x_val = float(p[1:])
                elif p.startswith("Y"):
                    y_val = float(p[1:])

            if x_val is None or y_val is None:
                continue  # bỏ nếu không đủ X, Y

            real_x = user_cordinate[0] + x_val
            real_y = user_cordinate[1] + y_val

            if line.startswith("G00"):  # pen up → từ vị trí trước
                if last_x is not None and last_y is not None:
                    # Lên cao từ vị trí cũ
                    output_file.write(f"arm.set_position(x={last_x:.4f}, y={last_y:.4f}, z={user_cordinate[2] + Z_offset:.4f}, roll=180, pitch=0, yaw=0, speed=50, wait=True)\n")
                    output_file.write("time.sleep(0.1)\n")
                # Đến vị trí mới (cao)
                output_file.write(f"arm.set_position(x={real_x:.4f}, y={real_y:.4f}, z={user_cordinate[2] + Z_offset:.4f}, roll=180, pitch=0, yaw=0, speed=50, wait=True)\n")
                output_file.write("time.sleep(0.1)\n")
                # Hạ xuống
                output_file.write(f"arm.set_position(x={real_x:.4f}, y={real_y:.4f}, z={user_cordinate[2]:.4f}, roll=180, pitch=0, yaw=0, speed=50, wait=True)\n")
                output_file.write("time.sleep(0.1)\n")

            elif line.startswith("G01"):  # pen down
                output_file.write(f"arm.set_position(x={real_x:.4f}, y={real_y:.4f}, z={user_cordinate[2]:.4f}, roll=180, pitch=0, yaw=0, speed=50, wait=True)\n")
                output_file.write("time.sleep(0.1)\n")

            last_x = real_x
            last_y = real_y

        # Kết thúc → về home & tắt servo
        output_file.write(f"arm.set_position(x={user_cordinate[0]}, y={user_cordinate[1]}, z={user_cordinate[2] + Z_offset}, roll=180, pitch=0, yaw=0, speed=50, wait=True)\n")
        output_file.write("time.sleep(0.1)\n")
        output_file.write("arm.set_position(x=261., y=0, z=258.3, roll=-180, pitch=0, yaw=0, speed=50, wait=True)\n")
        output_file.write("time.sleep(0.1)\n")
        output_file.write("arm.disconnect()\n")

    print("Conversion completed. Output written to 'converted_output.txt'.")
