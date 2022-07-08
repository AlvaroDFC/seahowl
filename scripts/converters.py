#!/usr/bin/env python3


from pathlib import Path
import json
import jsbeautifier
import numpy as np

options = jsbeautifier.default_options
options.indent_size = 2


def save_json(json_dict, path):
    mydirectory = Path(path).parent
    mydirectory.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        f.write(jsbeautifier.beautify(json.dumps(json_dict), options))


def convert_polar_file(filename, save_directory=None):
    filepath = Path(filename)
    airfoil = dict()
    with open(filepath, "r") as f:
        lines = f.readlines()
        coefficients = list()
        cl = list()
        cd = list()
        cm = list()
        for ii, line in enumerate(lines):
            words = line.split()
            if len(words) >= 2:
                if words[1] == "NumAlf":
                    npolars = int(words[0])
                    for jj in range(3, npolars + 3):
                        coeffs = lines[ii + jj].split()
                        coefficients.append(
                            [
                                float(coeffs[0]),
                                float(coeffs[1]),
                                float(coeffs[2]),
                                float(coeffs[3]),
                            ]
                        )
                if words[1] == "Re":
                    airfoil["reynolds_number"] = float(words[0])
    airfoil["header"] = ["alpha", "Cl", "Cd", "Cm"]
    airfoil["coefficients"] = coefficients

    # save to file
    if save_directory is not None:
        fullpath = Path(save_directory) / filename
        save_json(airfoil, fullpath)

    return airfoil


def convert_aerodyn_files(filename, blade_filename, save_directory=None):
    filepath = Path(filename)
    aerodyn_json = dict()
    polar_filenames = list()
    airfoil_files = list()
    with open(filepath, "r") as f:
        lines = f.readlines()
        for ii, line in enumerate(lines):
            words = line.split()
            if len(words) >= 2 and words[1] == "NumAFfiles":
                nfiles = int(words[0])
                for jj in range(1, nfiles + 1):
                    polar_filename = lines[ii + jj].split()[0]
                    polar_filename = polar_filename.replace('"', "")
                    polar_filename = polar_filename.replace("\n", "")
                    polar_filenames.append(str(polar_filename))
                    polar_filepath = filepath.parent / polar_filename
                    airfoil = convert_polar_file(
                        polar_filepath, save_directory=save_directory
                    )
                    airfoil_files.append(str(polar_filepath.with_suffix(".json")))

    filepath = Path(blade_filename)
    aerodyn_json = dict()
    polar_filenames = list()
    reference_points = list()
    with open(filepath, "r") as f:
        lines = f.readlines()
        for ii, line in enumerate(lines):
            words = line.split()
            if len(words) >= 2 and words[1] == "NumBlNds":
                npoints = int(words[0])
                for jj in range(3, nfiles + 3):
                    vals = lines[ii + jj].split()
                    point = dict()
                    point["coordinates"] = [
                        float(vals[1]),
                        float(vals[2]),
                        float(vals[0]),
                    ]
                    point["twist"] = float(vals[4])
                    point["chord"] = float(vals[5])
                    point["airfoil_file"] = airfoil_files[int(vals[6]) - 1]
                    reference_points.append(point)
    aerodyn_json["reference_points"] = reference_points

    # save to file
    if save_directory is not None:
        fullpath = Path(save_directory) / filepath.with_suffix(".json")
        save_json(aerodyn_json, fullpath)

    return aerodyn_json


def convert_beamdyn_file(filename, save_directory=None):
    filepath = Path(filename)
    beamdyn_json = dict()
    with open(filepath, "r") as f:
        lines = f.readlines()
        start_idx = 10
        nprops = (len(lines) - start_idx) / 15
        points = list()
        for ii in range(int(nprops)):
            idx = start_idx + ii * 15
            point = dict()
            point["fraction"] = lines[idx].split()[0]
            sm1 = lines[idx + 1].split()
            sm2 = lines[idx + 2].split()
            sm3 = lines[idx + 3].split()
            sm4 = lines[idx + 4].split()
            sm5 = lines[idx + 5].split()
            sm6 = lines[idx + 6].split()
            point["stiffness_matrix"] = [
                [
                    float(sm1[0]),
                    float(sm1[1]),
                    float(sm1[2]),
                    float(sm1[3]),
                    float(sm1[4]),
                    float(sm1[5]),
                ],
                [
                    float(sm2[0]),
                    float(sm2[1]),
                    float(sm2[2]),
                    float(sm2[3]),
                    float(sm2[4]),
                    float(sm2[5]),
                ],
                [
                    float(sm3[0]),
                    float(sm3[1]),
                    float(sm3[2]),
                    float(sm3[3]),
                    float(sm3[4]),
                    float(sm3[5]),
                ],
                [
                    float(sm4[0]),
                    float(sm4[1]),
                    float(sm4[2]),
                    float(sm4[3]),
                    float(sm4[4]),
                    float(sm4[5]),
                ],
                [
                    float(sm5[0]),
                    float(sm5[1]),
                    float(sm5[2]),
                    float(sm5[3]),
                    float(sm5[4]),
                    float(sm5[5]),
                ],
                [
                    float(sm6[0]),
                    float(sm6[1]),
                    float(sm6[2]),
                    float(sm6[3]),
                    float(sm6[4]),
                    float(sm6[5]),
                ],
            ]

            mm1 = lines[idx + 8].split()
            mm2 = lines[idx + 9].split()
            mm3 = lines[idx + 10].split()
            mm4 = lines[idx + 11].split()
            mm5 = lines[idx + 12].split()
            mm6 = lines[idx + 13].split()
            point["mass_matrix"] = [
                [
                    float(mm1[0]),
                    float(mm1[1]),
                    float(mm1[2]),
                    float(mm1[3]),
                    float(mm1[4]),
                    float(mm1[5]),
                ],
                [
                    float(mm2[0]),
                    float(mm2[1]),
                    float(mm2[2]),
                    float(mm2[3]),
                    float(mm2[4]),
                    float(mm2[5]),
                ],
                [
                    float(mm3[0]),
                    float(mm3[1]),
                    float(mm3[2]),
                    float(mm3[3]),
                    float(mm3[4]),
                    float(mm3[5]),
                ],
                [
                    float(mm4[0]),
                    float(mm4[1]),
                    float(mm4[2]),
                    float(mm4[3]),
                    float(mm4[4]),
                    float(mm4[5]),
                ],
                [
                    float(mm5[0]),
                    float(mm5[1]),
                    float(mm5[2]),
                    float(mm5[3]),
                    float(mm5[4]),
                    float(mm5[5]),
                ],
                [
                    float(mm6[0]),
                    float(mm6[1]),
                    float(mm6[2]),
                    float(mm6[3]),
                    float(mm6[4]),
                    float(mm6[5]),
                ],
            ]

            points.append(point)

    beamdyn_json["material_properties"] = points

    if save_directory is not None:
        fullpath = Path(save_directory) / filepath.with_suffix(".json")
        save_json(beamdyn_json, fullpath)

    return beamdyn_json


def merge_beamdyn2aerodyn(beamdyn_json, aerodyn_json):
    blade_length = aerodyn_json["reference_points"][-1]["coordinates"][2]
    aerodyn_fractions = list()
    beamdyn_fractions = list()
    for point in aerodyn_json["reference_points"]:
        aerodyn_fractions.append(point["coordinates"][2] / blade_length)
    for point in beamdyn_json["material_properties"]:
        beamdyn_fractions.append(point["fraction"])

    idx = 0
    for point in aerodyn_json["reference_points"]:
        afraction = float(point["coordinates"][2]) / blade_length
        bfraction0 = float(beamdyn_fractions[idx])
        bfraction1 = float(beamdyn_fractions[idx + 1])
        while afraction > bfraction1:
            idx += 1
            bfraction0 = float(beamdyn_fractions[idx])
            bfraction1 = float(beamdyn_fractions[idx + 1])
        bfraction_range = bfraction1 - bfraction0
        mm1 = np.array(beamdyn_json["material_properties"][idx]["mass_matrix"])
        mm2 = np.array(beamdyn_json["material_properties"][idx + 1]["mass_matrix"])
        sm1 = np.array(beamdyn_json["material_properties"][idx]["stiffness_matrix"])
        sm2 = np.array(beamdyn_json["material_properties"][idx + 1]["stiffness_matrix"])
        coeff1 = 1.0 - (afraction - bfraction0) / bfraction_range
        coeff2 = 1.0 - (bfraction1 - afraction) / bfraction_range
        mass_matrix = coeff1 * mm1 + coeff2 * mm2
        stiffness_matrix = coeff1 * sm1 + coeff2 * sm2

        point["mass_matrix"] = mass_matrix.tolist()
        point["coordinates"][1] = 0.0
        point["stiffness_matrix"] = stiffness_matrix.tolist()

    return aerodyn_json


if __name__ == "__main__":

    save_directory = "./converted"

    filename = "./IEA-15-240-RWT_AeroDyn15.dat"
    blade_filename = "./IEA-15-240-RWT_AeroDyn15_blade.dat"
    convert_aerodyn_files(filename, blade_filename, save_directory)

    beamdyn_filename = "./IEA-15-240-RWT_BeamDyn_blade.dat"
    convert_beamdyn_file(beamdyn_filename, save_directory)

    with open("./converted/IEA-15-240-RWT_BeamDyn_blade.json", "r") as f:
        beamdyn_json = json.load(f)
    with open("./converted/IEA-15-240-RWT_AeroDyn15_blade.json", "r") as f:
        aerodyn_json = json.load(f)

    merged_json = merge_beamdyn2aerodyn(beamdyn_json, aerodyn_json)
    save_json(merged_json, save_directory + "/blade.json")
