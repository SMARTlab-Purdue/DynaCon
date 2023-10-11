import json
import os
import argparse
from pathlib import Path
from datetime import datetime

import openai

import rospy
import re
from std_msgs.msg import String

import sys
sys.path.append(".")

from utils.convert_to_json import extract_function_text

objects_dict = {}
gen_plan = []
current_task = None

gpt_response_publisher = rospy.Publisher('/gpt_response', String, queue_size=10)

def LM(prompt, gpt_version, max_tokens=128, temperature=0, stop=None, logprobs=1, frequency_penalty=0):
    
    if "gpt" not in gpt_version:
        response = openai.Completion.create(model=gpt_version, 
                                            prompt=prompt, 
                                            max_tokens=max_tokens, 
                                            temperature=temperature, 
                                            stop=stop, 
                                            logprobs=logprobs, 
                                            frequency_penalty = frequency_penalty)
        
        return response, response["choices"][0]["text"].strip()
    
    else:
        response = openai.ChatCompletion.create(model=gpt_version, 
                                            messages=prompt, 
                                            max_tokens=max_tokens, 
                                            temperature=temperature, 
                                            frequency_penalty = frequency_penalty)
        
        return response, response["choices"][0]["message"]["content"].strip()


def set_api_key(openai_api_key):
    openai.api_key = Path(openai_api_key + '.txt').read_text()


def object_list_callback(data):
    
    global current_task
    
    objects_dict.clear()

    pattern = r'\[([\w\s]+), (\d+\.\d+)\]'
    matches = re.findall(pattern, data.data)
    
    for match in matches:
        object_name = match[0]
        distance = float(match[1])
        objects_dict[object_name] = distance

    add_prompt_string = '\n'.join([f"[{key}]" for key, value in objects_dict.items()])
    add_prompt = f"Mobile robot's desired goal is {task}\nObject list =\n{add_prompt_string}\nDesired object: "

    print(add_prompt)
    messages.append({"role": "user", "content": add_prompt})
    _, text = LM(messages, args.gpt_version, max_tokens=1000, stop=["def"], frequency_penalty=0.15)
    
    gpt_response_publisher.publish(text)
    print("🤖 ",text)
    gen_plan.append(add_prompt)
    gen_plan.append(text)


def ros_communication():
    rospy.init_node('ChatGPT_processor', anonymous=True)
    rospy.Subscriber("/object_list", String, object_list_callback)
    
    rospy.spin()



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--expt-name", type=str, required=True)
    parser.add_argument("--openai-api-key-file", type=str, default="api_key")
    parser.add_argument("--gpt-version", type=str, default="gpt-3.5-turbo", 
                        choices=['text-davinci-002', 'text-davinci-003', 'gpt-3.5-turbo', 'gpt-4'])
    parser.add_argument("--prompt-set", type=str, default="sample", 
                        choices=['sample'])
    parser.add_argument("--test-set", type=str, default="test_plans", 
                        choices=['test_unseen', 'test_seen', 'test_unseen_ambiguous', 'test_plans', 'env2'])

    # for random task examples, choose seed
    parser.add_argument("--seed", type=int, default=0)
    
    parser.add_argument("--prompt-task-examples-ablation", type=str, default="none", 
                         choices=['none', 'no_comments', "no_feedback", "no_comments_feedback"])
    
    parser.add_argument("--prompt-execution-env", type=str, default="terminal", 
                         choices=['terminal', 'webots', "virtualhome", "real"])

    parser.add_argument("--load-generated-plans", type=bool, default=False)

    parser.add_argument("--log-results", type=bool, default=True)
    
    args = parser.parse_args()

    set_api_key(args.openai_api_key_file)
    
    if not os.path.isdir(f"./logs/"):
        os.makedirs(f"./logs/")
    
    

    # convert the input python prompt to json
    json_prompts = extract_function_text(os.getcwd() + "/data/" + args.prompt_set + ".py") 

    prompt = ""

    for k, v in json_prompts.items():
        prompt += "\n\n" + json_prompts[k]

    test_tasks = []
    for file in os.listdir(f"./data/{args.test_set}"):
        with open(f"./data/{args.test_set}/{file}", 'r') as f:
            for line in f.readlines():
                test_tasks.append(line)

    print (prompt)
    print(f"\n----Test set tasks----\n{test_tasks}\nTotal: {len(test_tasks)} tasks\n")
    
    

    for task in test_tasks:

        current_task = task

        print(f"Desired goal is: {task}\n")
        
        
        if "gpt" not in args.gpt_version:
            prompt_task = "def {fxn}():".format(fxn = '_'.join(task.split(' ')))
            curr_prompt = f"{prompt}\n\n{prompt_task}\n\t"
            _, text = LM(curr_prompt, args.gpt_version, max_tokens=1000, stop=["def"], frequency_penalty=0.15)

        else:
            curr_prompt = f"{prompt}\n\n Desired goal: {task}\n\t"
            messages = [{"role": "user", "content": curr_prompt}]
            _, text = LM(messages, args.gpt_version, max_tokens=1000, stop=["def"], frequency_penalty=0.15)
    
    print("ChatGPT Ready")
    ros_communication()

    # save generated plan
    if args.log_results:
        line = {}
        now = datetime.now() # current date and time
        date_time = now.strftime("%m-%d-%Y-%H-%M-%S")

        print("")
        print(f"Saving generated plan at: {args.expt_name}_plans_{date_time}.json\n")
        with open(f"./logs/{args.expt_name}_plans_{date_time}.json", 'w') as f:
            for plan, task in zip(gen_plan, test_tasks):
                line[task] = plan
            json.dump(test_tasks, f)
            json.dump(gen_plan, f)
    
    
    


