#!/usr/bin/env python3


import argparse

import sys
import asyncio

try:
    import readline
except:
    pass  # readline not available

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

    parser.add_argument(
        "-m",
        "--model",
        help="Model file",
        type=argparse.FileType("r"),
    )

    parser.add_argument(
        "-v", "--verbose", help="Print the returned dict", action="store_true"
    )

    args = parser.parse_args()

    if (args.model and args.config) or (args.model and args.config):
        print("cant use model with config/nlu")
        sys.exit(1)

    from rasa.core.channels import UserMessage
    from rasa.core.processor import MessageProcessor
    from rasa.engine.constants import PLACEHOLDER_MESSAGE

    if args.model:
        _, model_metadata, graph_runner = MessageProcessor._load_model(args.model.name)
    else:
        if not args.nlu or not args.config:
            parser.print_usage()
            sys.exit(1)

        from rasa.model_training import train_nlu

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

            if args.verbose:
                print(f"\nfull output:\n  {kv}")

            print(f"\nintent ranking:")
            for intent in kv["intent_ranking"]:
                print(f"  - '{intent['name']}', conf:'{intent['confidence']:1.2f}'")

            print(
                f"\nInput: '{kv['text']}' intent: {kv['intent']['name']} conf: {kv['intent']['confidence']}"
            )
            for entity in kv["entities"]:
                text = f"  Entity:'{entity['value']}', type:'{entity['entity']}':{entity['confidence_entity']:1.2f}"
                if "role" in entity:
                    text += (
                        f", role:'{entity['role']}':{entity['confidence_role']:1.2f}"
                    )
                if "group" in entity:
                    text += (
                        f", group:'{entity['group']}':{entity['confidence_role']:1.2f}"
                    )
                print(text)
            print()
