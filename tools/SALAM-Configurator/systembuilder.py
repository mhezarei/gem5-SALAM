#!/usr/bin/env python3

# TODO: clean this file

# Import needed packages
import argparse
from http.client import FOUND
import os
import shutil
from unittest import expectedFailure
import config_parser

import yaml

# Define the imports of the gem5 script
imports = """import m5
from m5.objects import *
from m5.util import *
from configparser import ConfigParser
from HWAccConfig import *

"""
# L1 Cache defined here for now, need to add some more configurability to this
l1Cache = """class L1Cache(Cache):
\tassoc = 2
\ttag_latency = 2
\tdata_latency = 2
\tresponse_latency = 2
\tmshrs = 4
\ttgts_per_mshr = 20

\tdef __init__(self, size, options=None):
\t\tself.size = size
\t\tsuper(L1Cache, self).__init__()
\t\tpass

"""


def parse_cur_args():
    argparser = argparse.ArgumentParser(description="SALAM System Builder")
    argparser.add_argument(
        "--bench-path",
        help="Path to Benchmark Directory Relative to M5_PATH. e.g. gemm's bench path will be benchmarks/sys_validation/gemm",
        required=True,
    )
    argparser.add_argument(
        "--sys-name",
        help="Name of the generated python files. For a sys_name of gemm, the configurator will generate both fs_gemm.py & gemm.py in configs/SALAM).",
        required=True,
        default=None,
    )
    argparser.add_argument(
        "--config-name",
        help="Name of the configuration file in the root of the bench. Defaults to config.yml",
        required=False,
        default="config.yml",
    )
    argparser.add_argument(
        "--m5-path", help="Path to M5 Directory", required=False, default=None
    )
    return argparser.parse_args()


def parse_yaml(
    parent_config,
    base_address,
    working_dir: str,
    parent_path: str = None,
    hw_path: str = None,
):
    clusters = []

    # Load in each acc cluster and add it to the list
    for cluster_dict in parent_config:
        for list_type, params in cluster_dict.items():
            FOUND_SYS_PATH = False
            FOUND_HW_PATH = False
            FOUND_DEVICE = False
            if list_type == "acc_cluster":
                for param in params:
                    if "SysPath" in param:
                        FOUND_SYS_PATH = True
                        cur_config = open_yaml((working_dir + param["SysPath"]))
                        cur_path = working_dir + param["SysPath"]
                    elif "HWPath" in param:
                        FOUND_HW_PATH = True
                        hw_path = working_dir + param["HWPath"]
                    else:
                        FOUND_DEVICE = True
                        cur_config = parent_config
                        # Set the hard
                        if not FOUND_HW_PATH and not hw_path:
                            hw_path = parent_path
            if FOUND_SYS_PATH and FOUND_DEVICE:
                raise Exception(
                    "Found device definitions in a cluster with a path to another YAML file."
                )
            if FOUND_SYS_PATH:
                # Recursion Alert!
                base_address, temp_cluster = parse_yaml(
                    parent_config=cur_config,
                    base_address=base_address,
                    working_dir=working_dir,
                    parent_path=cur_path,
                    hw_path=hw_path,
                )
                clusters.extend(temp_cluster)
            elif FOUND_HW_PATH:
                raise Exception("HW Path should be defined with a System Config file")

        cluster_name = None
        dmas = []
        accs = []
        window_managers = []
        for device in cluster_dict["acc_cluster"]:
            if "Name" in device:
                cluster_name = device["Name"]
            if "DMA" in device:
                dmas.append(device)
            if "Accelerator" in device:
                accs.append(device)
            if "WindowManager" in device:
                window_managers.append(device)

        if not cluster_name:
            continue

        clusters.append(
            config_parser.AccCluster(
                name=cluster_name,
                dmas=dmas,
                accs=accs,
                window_managers=window_managers,
                base_address=base_address,
                working_dir=working_dir,
                config_path=parent_path,
                hw_config_path=hw_path,
            )
        )

        top_addr = clusters[-1].top_address
        base_address = top_addr + (64 - (int(top_addr) % 64))
        if (int(base_address) % 64) != 0:
            print("Address Alignment Error: " + hex(base_address))

    return base_address, clusters


def open_yaml(yml_path: str):
    stream = open(yml_path, "r")
    config = yaml.safe_load_all(stream)
    return config


def gen_config(clusters, config_path: str, file_name: str):
    # Write out config file
    with open(config_path + file_name + ".py", "w") as writer:
        writer.write(imports)
        writer.write(l1Cache)

        for cluster in clusters:
            for line in cluster.genConfig():
                writer.write(line + "\n")

            # Add cluster definitions here
            for dma in cluster.dmas:
                writeLines(writer, dma.genConfig())

            # Come back here later and see if you can get away with doing these at the same time (probably not)
            for acc in cluster.accs:
                writeLines(writer, acc.genDefinition())
            for wm in cluster.window_managers:
                writeLines(writer, wm.genDefinition())

            for acc in cluster.accs:
                writeLines(writer, acc.genConfig())
            for wm in cluster.window_managers:
                writeLines(writer, wm.genConfig())

        # Write cluster creation
        writer.write("def makeHWAcc(args, system):\n\n")
        for i in clusters:
            writer.write("	system." + i.name.lower() + " = AccCluster()" + "\n")
            writer.write(
                "	build" + i.name + "(args, system, system." + i.name.lower() + ")\n\n"
            )


def load_og_header(clusters, working_dir: str):
    begin, end = 0, 0

    # Read in existing header
    header_list = []
    for cluster in clusters:
        try:
            # f = open(working_dir + cluster.name  + "_" + args.headerName + ".h", 'r')
            f = open(working_dir + cluster.name + "_hw_defines.h", "r")
            oldHeader = f.readlines()

            for i in range(0, len(oldHeader)):
                if oldHeader[i] == "//BEGIN GENERATED CODE\n":
                    begin = i
                elif (
                    oldHeader[i] == "//END GENERATED CODE\n"
                    or oldHeader[i] == "//END GENERATED CODE"
                ):
                    end = i

            del oldHeader[begin : end + 1]
            header_list.append(oldHeader)
        except:
            print("No Header Found")
            emptyList = []
            header_list.append(emptyList)
    return header_list


def gen_header(header_list, clusters, working_dir: str):
    # Write out headers
    for current_header in header_list:
        for cluster in clusters:
            with open(working_dir + cluster.name + "_hw_defines.h", "w") as f:
                current_header.append("//BEGIN GENERATED CODE\n")
                current_header.append("//Cluster: " + cluster.name.upper() + "\n")
                for dma in cluster.dmas:
                    if dma.dmaType == "NonCoherent":
                        current_header.append("//" + dma.dmaType + "DMA" + "\n")
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_Flags "
                            + hex(dma.address)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_RdAddr "
                            + hex(dma.address + 1)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_WrAddr "
                            + hex(dma.address + 9)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_CopyLen "
                            + hex(dma.address + 17)
                            + "\n"
                        )
                    elif dma.dmaType == "Stream":
                        current_header.append("//" + dma.dmaType + "DMA" + "\n")
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_Flags "
                            + hex(dma.address)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_RdAddr "
                            + hex(dma.address + 4)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_WrAddr "
                            + hex(dma.address + 12)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_RdFrameSize "
                            + hex(dma.address + 20)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_NumRdFrames "
                            + hex(dma.address + 24)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_RdFrameBufSize "
                            + hex(dma.address + 25)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_WrFrameSize "
                            + hex(dma.address + 26)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_NumWrFrames "
                            + hex(dma.address + 30)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_WrFrameBufSize "
                            + hex(dma.address + 31)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_Stream "
                            + hex(dma.address + 32)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_Status "
                            + hex(dma.statusAddress)
                            + "\n"
                        )
                for acc in cluster.accs:
                    current_header.append("//Accelerator: " + acc.name.upper() + "\n")
                    current_header.append(
                        "#define " + acc.name.upper() + " " + hex(acc.address) + "\n"
                    )
                    for var in acc.variables:
                        if "Cache" in var.type:
                            continue
                        elif "Stream" in var.type:
                            current_header.append(
                                "#define " + var.name + " " + hex(var.address) + "\n"
                            )
                            current_header.append(
                                "#define "
                                + var.name
                                + "_Status "
                                + hex(var.statusAddress)
                                + "\n"
                            )
                        else:
                            current_header.append(
                                "#define " + var.name + " " + hex(var.address) + "\n"
                            )

                for wm in cluster.window_managers:
                    current_header.append("//WindowManager: " + wm.name.upper() + "\n")
                    current_header.append(
                        "#define " + wm.name.upper() + " " + hex(wm.pio_addr) + "\n"
                    )
                    for var in wm.variables:
                        if "Stream" in var.type:
                            current_header.append(
                                "#define " + var.name + " " + hex(var.address) + "\n"
                            )
                            current_header.append(
                                "#define "
                                + var.name
                                + "_Status "
                                + hex(var.statusAddress)
                                + "\n"
                            )
                        else:
                            current_header.append(
                                "#define " + var.name + " " + hex(var.address) + "\n"
                            )

                current_header.append("//END GENERATED CODE")
                f.writelines(current_header)
                current_header = []


def writeLines(writer, lines):
    for line in lines:
        writer.write("	" + line + "\n")


def main():
    args = parse_cur_args()

    # This requires M5_PATH to point to your gem5-SALAM directory
    M5_Path = os.getenv("M5_PATH")
    if not M5_Path:
        print("Looking for Path Argument from Command Line")
        if not args.m5_path:
            raise Exception("Path argument required when M5_PATH not set")
        M5_Path = args.path
        if not M5_Path:
            raise Exception("M5_PATH Not Found")

    # Set file information
    if args.sys_name == None:
        file_name = os.path.basename(os.path.normpath(args.sys_path))
    else:
        file_name = args.sys_name

    config_path = M5_Path + "/configs/SALAM/"
    working_dir = args.bench_path + "/"
    main_yml_path = working_dir + args.config_name

    # Set base addresses
    base_address = 0x10020000
    max_address = 0x13FFFFFF

    # Load in the YAML file
    config = open_yaml(yml_path=main_yml_path)

    # Parse YAML File
    base_address, clusters = parse_yaml(
        parent_config=config,
        base_address=base_address,
        working_dir=working_dir,
        parent_path=main_yml_path,
    )

    # Generate SALAM Config
    gen_config(clusters=clusters, config_path=config_path, file_name=file_name)

    # Parse original header for custom code
    header_list = load_og_header(clusters=clusters, working_dir=working_dir)

    # Make the header files with custom code
    gen_header(header_list=header_list, clusters=clusters, working_dir=working_dir)

    # Generate full system file
    shutil.copyfile(
        M5_Path + "/tools/SALAM-Configurator/fs_template.py",
        config_path + "fs_" + file_name + ".py",
    )
    f = open(config_path + "fs_" + file_name + ".py", "r")
    fullSystem = f.readlines()

    # smarter way of doing this
    # fullSystem[65] = "import " + file_name
    # fullSystem[229] = "        " + file_name + ".makeHWAcc(args, test_sys)\n"
    fullSystem = [line.replace("TEMPLATE", file_name) for line in fullSystem]

    f = open(config_path + "fs_" + file_name + ".py", "w")
    f.writelines(fullSystem)
    # Warn if the size is greater than allowed
    if clusters[-1].top_address > max_address:
        print("WARNING: Address range is greater than defined for gem5")


if __name__ == "__main__":
    main()
