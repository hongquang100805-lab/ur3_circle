import json

class TaskValidator:
    def __init__(self):
        self.allowed_skills = ["home", "pick", "place"]
        self.allowed_objects = ["red_cube", "yellow_cube", "blue_cube"]
        self.allowed_zones = ["zone_a", "zone_b", "zone_c"]

    def validate(self, plan_json):
        try:
            if isinstance(plan_json, str):
                data = json.loads(plan_json)
            else:
                data = plan_json
                
            if "plan" not in data or not isinstance(data["plan"], list):
                return False, "INVALID_PLAN"
                
            for step in data["plan"]:
                if "skill" not in step:
                    return False, "INVALID_PLAN"
                
                skill = step["skill"]
                if skill not in self.allowed_skills:
                    return False, "INVALID_SKILL"
                    
                if skill == "pick":
                    if "object" not in step:
                        return False, "INVALID_PLAN"
                    if step["object"] not in self.allowed_objects:
                        return False, "INVALID_OBJECT"
                        
                elif skill == "place":
                    if "object" not in step or "zone" not in step:
                        return False, "INVALID_PLAN"
                    if step["object"] not in self.allowed_objects:
                        return False, "INVALID_OBJECT"
                    if step["zone"] not in self.allowed_zones:
                        return False, "INVALID_ZONE"
                        
            return True, "PLAN VALID"
        except json.JSONDecodeError:
            return False, "JSON_ERROR"
        except Exception:
            return False, "VALIDATION_FAILED"
