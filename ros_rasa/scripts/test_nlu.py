#!/usr/bin/env python3

import os
import sys
import threading
import argparse

# ROS
import rospy

from clf_speech_msgs.srv import Understand
from clf_speech_msgs.msg import Entity


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

    success = True

    with args.file as file:
        for line in file:
            input = line.split("=")
            resp = understand(input[0])
            if args.check:
                ok = True
                error = ""
                intent = input[1].strip()
                if intent != resp.nlu.intent:
                    ok = False
                    error += f"wrong intent: '{resp.nlu.intent}' should be '{intent}', "
                entities = input[2].split(";") if len(input) > 2 else []
                resp_entities = resp.nlu.entities


                for ent in entities:
                    ent = ent.strip()
                    if not ent:
                        ## Empty string
                        pass

                    entity = Entity()
                   
                    fields = ent.split(":") 
                    entity.key, entity.value = fields[:2]
                    entity.role = fields[2] if len(fields) > 2 else "" 
                    entity.group = fields[3] if len(fields) > 3 else -1

                    if entity in resp_entities:
                        resp_entities.remove(entity)
                    else:
                        estring = f"{entity.key}:{entity.value}:{entity.role}:{entity.group}"
                        ok = False
                        error+=f"missing entity: {estring}, "

                for ent in resp_entities:
                    ok = False  
                    estring = f"{ent.key}:{ent.value}:{ent.role}:{ent.group}"
                    error += f"extra entity: {estring}, "
                if ok:
                    print(f"OK. input {resp.nlu.text}")
                else:
                    success = False
                    print(f"ERROR. input: '{resp.nlu.text}' errors: {error}")

            else:
                ents = f";".join(map(lambda e: f"{e.key}:{e.value}:{e.role}:{e.group}", resp.nlu.entities))
                print(f"{resp.nlu.text}={resp.nlu.intent}={ents}" )

    if not success:
        sys.exit("Failed check")