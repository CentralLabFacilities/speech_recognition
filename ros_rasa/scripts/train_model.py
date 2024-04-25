#!/usr/bin/env python3

import argparse
import sys
import os


def dir_path(string):
    if os.path.isdir(string):
        return string
    else:
        raise NotADirectoryError(string)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Train Rasa Model")
    parser.add_argument(
        "-n",
        "--nlu",
        help="NLU file",
        type=argparse.FileType("r"),
    )

    parser.add_argument(
        "-m",
        "--model_name",
        help="model name",
    )

    parser.add_argument(
        "-c",
        "--config",
        help="Config file",
        type=argparse.FileType("r"),
    )

    parser.add_argument(
        "-o",
        "--model_dir",
        help="Model output dir",
        type=dir_path,
        default="./models",
    )

    args = parser.parse_args()

    if not args.nlu or not args.config or not args.model_name:
        parser.print_usage()
        sys.exit(1)

    print(
        f"""\
using nlu: '{args.nlu.name}'
config: '{args.config.name}'
model_dir:{args.model_dir}
"""
    )

    from rasa.model_training import train_nlu

    mod = train_nlu(
        nlu_data=args.nlu.name,
        config=args.config.name,
        output=args.model_dir,
        fixed_model_name=args.model_name,
    )
