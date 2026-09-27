"""
How to:
turn on robot, get into motion state (L2 + B; L2 + UP; R2 + A on remote control)
then run python main.py in unitree_sdk conda environment



audio output -> simon says or nothing
audio output -> <action> 

action output -> right or wrong or nothing
wait
"""

import random
from time import sleep
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize

import sys
# caution: path[0] is reserved for script path (or '' in REPL)
sys.path.insert(1, "../new_highlevel_milena_keno")

#from execute_new_commands import Executer
#from execute_from_positions import Executer
from execute_from_positions import Executer 
from config import NETWORK_CARD_NAME


ROUNDS = 5 #number of rounds
MOVEMENT_NAMES = ["salute",
                "capitulate",
                "drumming"]



def randomizer(round):
    i = random.random()
    if i <= round / ROUNDS:
        return 1
    else:
        return 0

def main(Executer, audio_client):
    """inp= input("Should I explain the rules? (Yes / No)")
    if inp == "Yes":
        print("Google it!")
    elif inp == "No":
        print("Okaaaayyy, let's go!")
    else:
        print("Too dumn to write yes or no?!")"""

    for round in range(ROUNDS):
        
        # say "simon says" (1) or not (0)
        #add simulated annealing later?
        simon_rand = random.randint(0,1) 
        # simon_rand = randomizer(round)  

        # add any system to the random choice of action?
        movement_choice = random.randint(0, len(MOVEMENT_NAMES)-1)
        movement_name = MOVEMENT_NAMES[movement_choice]

        # add feature that adds random/ false moves to trick players
        movement_code = MOVEMENT_NAMES[movement_choice]

        if simon_rand:
            audio_client.TtsMaker(f"Simon says: {movement_name}", 1)
            sleep(0.3) # allow extra time for "simon says"
            #print(f"Simon says: {movement_name}")
        else:
            audio_client.TtsMaker(movement_name, 1)
            #print(movement_name)


        Executer.set_target_position(movement_code)
        #print(f"I'm moving {movement_code}")

        while not Executer.done:
            # keep this one short for inbetween movements
            # needed to make sure it waits for movement to be done
            sleep(0.25)
        
        # kick people out here?

    print("Game is done!")
    audio_client.TtsMaker("Game over!", 1)
  

if __name__ == '__main__':

    print("WARNING: Please ensure there are no obstacles around the robot while running movements")
    input("Press Enter to continue...")

    ChannelFactoryInitialize(0, NETWORK_CARD_NAME)
    
    # motion control client
    Executer = Executer()
    Executer.Init()
    Executer.Start()
    Executer.set_target_position("neutral")

    # audio client
    audio_client = AudioClient() 
    ret = audio_client.GetVolume()
    print("debug GetVolume: ",ret) 
    audio_client.SetVolume(85)
    audio_client.SetTimeout(10.0)
    audio_client.Init()

    while not Executer.done: # wait for neutral position
        sleep(1)

    # run main
    main(Executer, audio_client)
    
    # return to neutral position
    Executer.set_target_position("neutral") # make sure it ends in a neutral position
    while not Executer.done:
        sleep(1)


    # exit sequence
    Executer.give_control_back()

    if Executer.done and not Executer.script_control: 
        print("Robot has regained control. You should be able to use the remote control again.") 



