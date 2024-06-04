#!/usr/bin/env python3

import os
import sys
import threading
import argparse

# ROS
import rospy

from clf_speech_msgs.srv import Understand


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('file', type=argparse.FileType('r'))
    parser.add_argument('-c', '--check',
                    action='store_true')   
    args = parser.parse_args()

    # Start ROS node
    rospy.init_node("rasa_tester")
    understand = rospy.ServiceProxy('/rasa/understand', Understand)

    if args.check:
        print("checking FILE...")

    with args.file as file:
        for line in file:
            input = line.split("=")
            resp = understand(input[0])
            if args.check:
                ok = True
                error = "ERROR:"
                intent = input[1].strip()
                if intent != resp.nlu.intent:
                    ok = False
                    error += f"wrong intent: '{resp.nlu.intent}' should be '{intent}'"
                entities = input[2].split(";") if len(input) > 2 else []
                resp_entities = resp.nlu.entities
                for ent in entities:
                    if not ent:
                        ## Empty string
                        pass
                    else:
                        pass
                for ent in resp_entities:
                    ok = False  
                    error += f"extra entity: {ent}"
                if ok:
                    print("OK")
                else:
                    print(error)

            else:
                ents = f";".join(map(lambda e: f"{e.key}:{e.value}:{e.role}:{e.group}", resp.nlu.entities))
                print(f"{resp.nlu.text}={resp.nlu.intent}={ents}" )

