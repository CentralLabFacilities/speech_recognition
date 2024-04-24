from rasa.engine.constants import PLACEHOLDER_MESSAGE
from rasa.core.channels import UserMessage
from rasa.core.processor import MessageProcessor


class RasaNLU(object):
    def __init__(self, model, train=False, nlu_file="", config=""):
        """
                Parameters
        ----------
        model : str
            path to the modelfile or folder when training
        """

        if train:
            from rasa.model_training import train_nlu

            mod = train_nlu(nlu_data=nlu_file, config=config, output=model)

        _, self.model_metadata, self.graph_runner = MessageProcessor._load_model(model)

    def inference(self, text):
        # print(f"inference on '{text}'")

        message = UserMessage(text)
        results = self.graph_runner.run(
            inputs={PLACEHOLDER_MESSAGE: [message]},
            targets=[self.model_metadata.nlu_target],
        )

        return results[self.model_metadata.nlu_target][0].as_dict()
