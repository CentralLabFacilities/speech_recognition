#!/usr/bin/env python3


import argparse

import sys
import asyncio

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test Rasa NLU Data")
    parser.add_argument(
        "-n",
        "--nlu",
        help="NLU file",
        type=argparse.FileType("r"),
    )

    parser.add_argument(
        "-c",
        "--config",
        help="Config file",
        type=argparse.FileType("r"),
    )

    args = parser.parse_args()

    if not args.nlu or not args.config:
        parser.print_usage()
        sys.exit(1)

    from rasa.model_training import train_nlu
    from rasa.engine.constants import PLACEHOLDER_MESSAGE
    from rasa.core.channels import UserMessage
    from rasa.core.processor import MessageProcessor

    mod = train_nlu(
        nlu_data=args.nlu.name,
        config=args.config.name,
        output="/tmp/rasa",
    )

    print("Model finished training")
    _, model_metadata, graph_runner = MessageProcessor._load_model(mod)

    print(
"""
########################
Model Trained and Loaded
########################
"""
    )

    while True:
        var = input("Please enter something: ")
        message = UserMessage(var)

        results = graph_runner.run(
            inputs={PLACEHOLDER_MESSAGE: [message]}, targets=[model_metadata.nlu_target]
        )
        for output in results[model_metadata.nlu_target]:
            kv = output.as_dict()
            print(f"full output: {kv}")
            print(
                f"\nInput: '{kv['text']}' intent: {kv['intent']['name']} conf: {kv['intent']['confidence']}"
            )
            for entity in kv["entities"]:
                if "role" in entity:
                    print(
                        f"  Entity:'{entity['value']}', type:'{entity['entity']}':{entity['confidence_entity']:1.2f}, role:'{entity['role']}':{entity['confidence_role']:1.2f}"
                    )
                else:
                    print(
                        f"  Entity:'{entity['value']}', type:'{entity['entity']}':{entity['confidence_entity']:1.2f}"
                    )
            print()
