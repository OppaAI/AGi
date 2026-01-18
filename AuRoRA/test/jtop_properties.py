#!/usr/bin/env python
from jtop import jtop


if __name__ == "__main__":

    print("All accessible jtop properties")

    with jtop() as jetson:
        # boards
        print('*** board ***')
        print(jetson.board)
        # jetson.ok() will provide the proper update frequency
        #while jetson.ok():
        # CPU
        print('*** CPUs ***')
        print(jetson.cpu)
        # CPU
        print('*** Memory ***')
        print(jetson.memory)
        # GPU
        print('*** GPU ***')
        print(jetson.gpu)
        # Engines
        print('*** engine ***')
        print(jetson.engine)
        # nvpmodel
        print('*** NV Power Model ***')
        print(jetson.nvpmodel)
        # jetson_clocks
        print('*** jetson_clocks ***')
        print(jetson.jetson_clocks)
        # Status disk
        print('*** disk ***')
        print(jetson.disk)
        # Status fans
        print('*** fan ***')
        print(jetson.fan)
        # uptime
        print('*** uptime ***')
        print(jetson.uptime)
        # local interfaces
        print('*** local interfaces ***')
        print(jetson.local_interfaces)
        # Temperature
        print('*** temperature ***')
        print(jetson.temperature)
        # Power
        print('*** power ***')
        print(jetson.power)
        # Processes
        print('*** processes ***')
        print(jetson.processes)
        # Native ID
        print('*** native ID ***')
        print(jetson.native_id)
# EOF