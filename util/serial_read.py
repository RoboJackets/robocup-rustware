#
# Python Script to read incoming serial data from the robot (motion control values) and parse the values into a csv
#

import re
from typing import List
import serial
import argparse
import polars as pl


def collect_data_from_serial(port: str) -> List[str]:
    ser = serial.Serial(port, timeout=3.0)
    
    print(f"Connected To: {ser.portstr}")
    
    data = []
    for line in ser.readlines():
        line = line.decode("utf-8")
        if "MotionControlReading" in line:
            data.append(line)
        elif "DONE" in line:
            print("Done Reporting Data")
            break
    return data

def parse_data(data_lines: List[str]) -> pl.DataFrame:
    data = {
        "accel_x": [],
        "accel_y": [],
        "gyro_z": [],
        "encoder_values": [],
        "delta_t": []
    }
    for line in data_lines:
        try:
            inner_data = re.findall(r'\{(.*?)\}', line)[0]
        except:
            print(f"Accidentally Found: {line}")
            continue
        accel_x = int(re.findall(r'accel_x: (.*?),', inner_data)[0].strip())
        accel_y = int(re.findall(r'accel_y: (.*?),', inner_data)[0].strip())
        gyro_z = int(re.findall(r'gyro_z: (.*?),', inner_data)[0].strip())
        encoder_values = re.findall(f'encoder_values: \[(.*?)\],', inner_data)[0].strip()
        encoder_values = [int(v) for v in encoder_values.split(", ")]
        delta_t = int(re.findall(r'delta_t: (.*?),', inner_data)[0].strip())
        data["accel_x"].append(accel_x)
        data["accel_y"].append(accel_y)
        data["gyro_z"].append(gyro_z)
        data["encoder_values"].append(encoder_values)
        data["delta_t"].append(delta_t)
    return pl.DataFrame(
        data,
        schema=[
            ("accel_x", pl.Int32),
            ("accel_y", pl.Int32),
            ("gyro_z", pl.Int32),
            ("encoder_values", pl.Array(pl.UInt8, 10)),
            ("delta_t", pl.UInt64),
        ]
    )

def main(port: str, out: str):
    print("Collecting Data From Serial")
    data_lines = collect_data_from_serial(port)
    print("Parsing Data")
    data = parse_data(data_lines)
    print("Exporting Data to CSV")
    data.write_csv(out)
    print("Complete")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", help="The COM Port to Listen To", default="tty/ACM0")
    parser.add_argument("-o", "--out", help="File to output data to", required=True)
    args = parser.parse_args()
    main(args.port, args.out)