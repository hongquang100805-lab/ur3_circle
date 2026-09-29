import sys
import rclpy
import json
import os
from ament_index_python.packages import get_package_share_directory

from .task_validator import TaskValidator
from .robot_skills import RobotSkills
from .llm_planner import LLMPlanner

def main(args=None):
    rclpy.init(args=args)
    
    pkg_share = get_package_share_directory('ur3_llm_control')
    scene_path = os.path.join(pkg_share, 'config', 'scene.yaml')
    student_config_path = os.path.join(pkg_share, 'config', 'student_config.yaml')
    
    planner = LLMPlanner()
    validator = TaskValidator()
    skills = RobotSkills(scene_path)
    
    import yaml
    with open(student_config_path, 'r') as f:
        student_config = yaml.safe_load(f)
    
    try:
        while rclpy.ok():
            command = input("\nENTER COMMAND (or 'quit'): ")
            if command.lower() in ['quit', 'exit', 'q']:
                break
                
            print(f"\nUSER COMMAND:\n{command}")
            
            # 1. LLM Planning
            print("\nGenerating plan...")
            plan_json_str = planner.plan(command)
            if not plan_json_str:
                print("LLM_ERROR")
                continue
                
            print(f"\nLLM PLAN:\n{plan_json_str}")
            
            # 2. Validation
            print("\nVALIDATION:")
            is_valid, msg = validator.validate(plan_json_str)
            print(msg)
            
            if not is_valid:
                print("TASK FAILED (Validation)")
                continue
                
            # 3. Execution
            print("\nEXECUTION:")
            plan_data = json.loads(plan_json_str)
            
            success = True
            for step in plan_data["plan"]:
                skill_name = step["skill"]
                
                if skill_name == "home":
                    res = skills.home()
                    print(f"home() ................. {res}")
                elif skill_name == "pick":
                    obj = step["object"]
                    res = skills.pick(obj)
                    print(f"pick({obj}) ........ {res}")
                elif skill_name == "place":
                    obj = step["object"]
                    zone = step["zone"]
                    res = skills.place(obj, zone)
                    print(f"place({obj}, {zone}) ........ {res}")
                    
                if res != "SUCCESS":
                    success = False
                    break
                    
            if success:
                print("\nTASK SUCCESS")
            else:
                print("\nTASK FAILED")
                
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"ERROR: {e}")
    finally:
        planner.destroy_node()
        skills.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
