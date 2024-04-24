from rasa.engine.constants import PLACEHOLDER_MESSAGE
from rasa.core.channels import UserMessage
from rasa.core.processor import MessageProcessor

import asyncio

if __name__ == "__main__":
    train = False
    if train:
        from rasa.model_training import train_nlu

        mod = train_nlu(
            nlu_data="data/nlu.yml",
            config=f"config/test.yml",
            output="models",
        )
    else:
        mod = "models"
    _, model_metadata, graph_runner = MessageProcessor._load_model(mod)

    while True:
        var = input("Please enter something: ")
        message = UserMessage(var)

        results = graph_runner.run(
            inputs={PLACEHOLDER_MESSAGE: [message]},
            targets=[model_metadata.nlu_target],
            fixed_model_name="MODEL_NAME",
        )
        for output in results[model_metadata.nlu_target]:
            kv = output.as_dict()
            print(kv)
            # for intent in kv["intent_ranking"]:
            # 	print(f"{intent['name']} conf: {intent['confidence']}")
            print(
                f"\nInput: '{kv['text']}' intent: {kv['intent']['name']} conf: {kv['intent']['confidence']}"
            )
            for entity in kv["entities"]:
                print(f"Entity: '{entity['value']}' type '{entity['entity']}'")
