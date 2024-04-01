import serial
import csv


def read_data():
    ser = serial.Serial('ttyACM0')
    with open('data.csv', mode='w') as file:
            writer = csv.writer(file, delimiter=',')
            header = ["smt1, smt2, smt3"]
            writer.writerow(header)
    while True:
        rawdata = (ser.readline().decode('ascii'))

        with open('data.csv', mode='w') as file:
            writer.writerow(rawdata)