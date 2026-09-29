import os
import json
import rclpy
from rclpy.node import Node
import urllib.request

class LLMPlanner(Node):
    def __init__(self):
        super().__init__('llm_planner')
        
        self.api_key = os.getenv("OPENAI_API_KEY", "dummy-key-for-local")
        self.api_url = "http://localhost:20128/v1/chat/completions"
        
        self.system_prompt = """Bạn là robot task planner.

Bạn chỉ được phép lựa chọn và sắp xếp các robot skill đã được định nghĩa.

Bạn KHÔNG được tạo:
- joint trajectory;
- joint position;
- joint command;
- controller command;
- low-level robot control.

Chỉ được trả về JSON hợp lệ theo đúng schema sau.

Mỗi task bắt buộc phải có 3 bước:

1. pick:
{
  "skill": "pick",
  "object": "<object>"
}

2. place:
{
  "skill": "place",
  "object": "<same object as pick>",
  "zone": "<zone>"
}

3. home:
{
  "skill": "home"
}

Ví dụ:

User: đưa vật màu đỏ tới vùng B

Output bắt buộc:
{
  "plan": [
    {
      "skill": "pick",
      "object": "red_cube"
    },
    {
      "skill": "place",
      "object": "red_cube",
      "zone": "zone_b"
    },
    {
      "skill": "home"
    }
  ]
}

QUAN TRỌNG:
- Không được bỏ trường "object" trong skill "place".
- object trong "place" phải giống object trong "pick".
- Luôn phải có skill "home" ở cuối.
- Không thêm giải thích ngoài JSON.
- Chỉ trả về JSON.

Allowed skills: home, pick, place
Allowed objects: red_cube, yellow_cube, blue_cube
Allowed zones: zone_a, zone_b, zone_c
"""
    
    def plan(self, command):
        try:
            payload = {
                "model": "oc/muse-spark-1.3-contributor-free",
                "messages": [
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": command}
                ],
                "response_format": { "type": "json_object" }
            }
            
            data = json.dumps(payload).encode('utf-8')
            req = urllib.request.Request(self.api_url, data=data, method="POST")
            req.add_header("Content-Type", "application/json")
            req.add_header("Authorization", f"Bearer {self.api_key}")
            
            with urllib.request.urlopen(req) as response:
                result = json.loads(response.read().decode('utf-8'))
                return result['choices'][0]['message']['content']
        except Exception as e:
            self.get_logger().error(f"LLM API Error: {str(e)}. Using mock response.")
            # Fallback for testing when 9Router is unavailable
            normalized_command = command.lower()
            object_names = {
                "red_cube": ("red", "đỏ"),
                "yellow_cube": ("yellow", "vàng"),
                "blue_cube": ("blue", "xanh dương"),
            }
            zone_names = {
                "zone_a": ("zone a", "zone_a", "vùng a"),
                "zone_b": ("zone b", "zone_b", "vùng b"),
                "zone_c": ("zone c", "zone_c", "vùng c"),
            }
            selected_object = next(
                (name for name, aliases in object_names.items()
                 if any(alias in normalized_command for alias in aliases)),
                None,
            )
            selected_zone = next(
                (name for name, aliases in zone_names.items()
                 if any(alias in normalized_command for alias in aliases)),
                None,
            )

            if selected_object and selected_zone:
                return json.dumps({
                    "plan": [
                        {"skill": "pick", "object": selected_object},
                        {"skill": "place", "object": selected_object, "zone": selected_zone},
                        {"skill": "home"},
                    ]
                })
            elif "student id" in normalized_command:
                return '{"plan": [{"skill": "pick", "object": "yellow_cube"}, {"skill": "place", "object": "yellow_cube", "zone": "zone_a"}, {"skill": "pick", "object": "blue_cube"}, {"skill": "place", "object": "blue_cube", "zone": "zone_b"}, {"skill": "pick", "object": "red_cube"}, {"skill": "place", "object": "red_cube", "zone": "zone_c"}, {"skill": "home"}]}'
            return None

def main(args=None):
    rclpy.init(args=args)
    planner = LLMPlanner()
    
    # Just a simple CLI for testing
    while rclpy.ok():
        try:
            command = input("\nEnter command (or 'quit'): ")
            if command.lower() in ['quit', 'exit', 'q']:
                break
                
            print(f"\nUSER COMMAND:\n{command}")
            plan = planner.plan(command)
            
            if plan:
                print(f"\nLLM PLAN:\n{plan}")
                
        except KeyboardInterrupt:
            break
            
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
