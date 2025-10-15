import csv
from pathlib import Path
import orca_auv_thruster_pkg
import bisect


class ThrusterLookupTable:

    def __init__(self):

        self._data = []

        self._load_data()

    def _load_data(self):
        with open(str(Path(orca_auv_thruster_pkg.__file__).parent) + '/thruster_lookup_table_16V.csv') as lookup_table_file:
            reader = csv.reader(lookup_table_file)
            header = next(reader)   # skip header
            for row in reader:
                self._data.append({"PWM_us": int(row[0]), "Force_Kg_f": float(row[1])})

    def get_pwm_signal_us(self, output_force_N):
        STANDARD_GRAVITY_M_PER_S_SQUARED = 9.80665
        output_force_kg_f = output_force_N / STANDARD_GRAVITY_M_PER_S_SQUARED

        if output_force_kg_f == 0:
            return 1500

        index = bisect.bisect_right(self._data, output_force_kg_f, key=lambda x: x["Force_Kg_f"]) # assuming data is sorted

        if index > len(self._data) - 1:
            return self._data[-1]["PWM_us"]

        if index == 0:
            return self._data[0]["PWM_us"]

        return int(self._data[index - 1]["PWM_us"]
                + (self._data[index]["PWM_us"] - self._data[index - 1]["PWM_us"])
                * (output_force_kg_f - self._data[index - 1]["Force_Kg_f"])
                / (self._data[index]["Force_Kg_f"] - self._data[index - 1]["Force_Kg_f"])) # linear interpolation


if __name__ == '__main__':
    thruster_lookup_table = ThrusterLookupTable()

    for force_N in range(-100, 100):
        print("force_N: " + str(force_N) + ", pwm_signal_us: " + str(thruster_lookup_table.get_pwm_signal_us(force_N)))
