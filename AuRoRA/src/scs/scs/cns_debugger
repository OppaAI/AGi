#!/usr/bin/env python3
"""
Grace CNS Bridge Debugger (Simple Version)
===========================================

Monitors all ROS topics without external dependencies
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import base64
from datetime import datetime
import sys


class GraceDebugger(Node):
    
    def __init__(self):
        super().__init__('grace_debugger')
        
        print("=" * 80)
        print("GRACE CNS BRIDGE DEBUGGER")
        print("=" * 80)
        print()
        
        self.text_count = 0
        self.image_count = 0
        self.response_count = 0
        self.current_stream = []
        
        # Subscribe to all topics
        self.text_sub = self.create_subscription(
            String, '/cns/neural_input', self.on_text_input, 10)
        
        self.image_sub = self.create_subscription(
            String, '/cns/image_input', self.on_image_input, 10)
        
        self.response_sub = self.create_subscription(
            String, '/gce/response', self.on_response, 10)
        
        print("Subscribed to topics:")
        print("  - /cns/neural_input (text)")
        print("  - /cns/image_input (images)")
        print("  - /gce/response (responses)")
        print()
        print("Listening... (Ctrl+C to stop)")
        print()
        print("-" * 80)
        print()
    
    def on_text_input(self, msg):
        """Monitor text input"""
        self.text_count += 1
        ts = datetime.now().strftime("%H:%M:%S")
        
        print("=" * 80)
        print(f"[{ts}] TEXT INPUT #{self.text_count}")
        print("=" * 80)
        
        # Try JSON parse
        try:
            data = json.loads(msg.data)
            print("WARNING: Received JSON (should be plain text!)")
            print(f"JSON: {json.dumps(data, indent=2)}")
            
            if 'text' in data:
                print(f"\nText field: {data['text']}")
            if 'web_search' in data:
                print(f"Web search flag: {data['web_search']}")
        
        except json.JSONDecodeError:
            print("OK: Plain text format (correct)")
            print(f"Message: {msg.data}")
            
            # Check for search keywords
            keywords = ['search', 'find', 'lookup', 'google', 'web']
            if any(k in msg.data.lower() for k in keywords):
                print("\n>>> WEB SEARCH LIKELY TO TRIGGER (keyword detected)")
        
        print()
        print("-" * 80)
        print()
    
    def on_image_input(self, msg):
        """Monitor image input"""
        self.image_count += 1
        ts = datetime.now().strftime("%H:%M:%S")
        
        print("=" * 80)
        print(f"[{ts}] IMAGE INPUT #{self.image_count}")
        print("=" * 80)
        
        try:
            data = json.loads(msg.data)
            print("JSON parsed successfully")
            
            # Check prompt
            if 'prompt' in data:
                print(f"Prompt: {data['prompt']}")
            else:
                print("ERROR: Missing 'prompt' field!")
            
            # Check image
            if 'image' in data:
                img = data['image']
                print(f"\nImage data length: {len(img)} chars")
                
                # Check data URL format
                if img.startswith('data:image'):
                    print("OK: Data URL format detected")
                    
                    # Extract type
                    if 'jpeg' in img[:50]:
                        print("Format: JPEG")
                    elif 'png' in img[:50]:
                        print("Format: PNG")
                    
                    # Validate base64
                    try:
                        parts = img.split(',', 1)
                        if len(parts) == 2:
                            decoded = base64.b64decode(parts[1], validate=True)
                            size_kb = len(decoded) / 1024
                            print(f"OK: Valid base64, size: {size_kb:.1f} KB")
                            
                            if size_kb > 5120:
                                print("WARNING: Image > 5MB!")
                            
                            # Check signature
                            if decoded[:2] == b'\xff\xd8':
                                print("OK: Valid JPEG signature")
                            elif decoded[:8] == b'\x89PNG\r\n\x1a\n':
                                print("OK: Valid PNG signature")
                            else:
                                print("WARNING: Unknown image signature")
                        else:
                            print("ERROR: Malformed data URL")
                    except Exception as e:
                        print(f"ERROR: Base64 invalid - {e}")
                else:
                    print("WARNING: Not a data URL (missing 'data:image' prefix)")
            else:
                print("ERROR: Missing 'image' field!")
            
            # Show structure
            print(f"\nJSON structure:")
            for key in data.keys():
                if key == 'image':
                    print(f"  - {key}: <{len(data[key])} chars>")
                else:
                    print(f"  - {key}: {data[key]}")
        
        except json.JSONDecodeError as e:
            print(f"ERROR: JSON parse failed - {e}")
            print(f"Raw data preview: {msg.data[:200]}...")
        
        except Exception as e:
            print(f"ERROR: {e}")
        
        print()
        print("-" * 80)
        print()
    
    def on_response(self, msg):
        """Monitor responses"""
        ts = datetime.now().strftime("%H:%M:%S")
        
        try:
            data = json.loads(msg.data)
            
            if data.get('type') == 'start':
                self.response_count += 1
                self.current_stream = []
                print("=" * 80)
                print(f"[{ts}] RESPONSE #{self.response_count} START")
                print("=" * 80)
                print(data.get('content', ''), end='', flush=True)
                self.current_stream.append(data.get('content', ''))
            
            elif data.get('type') == 'delta':
                content = data.get('content', '')
                print(content, end='', flush=True)
                self.current_stream.append(content)
            
            elif data.get('type') == 'done':
                print("\n")
                print(f"OK: Stream complete ({len(''.join(self.current_stream))} chars)")
                print("-" * 80)
                print()
                self.current_stream = []
            
            elif data.get('type') == 'error':
                print(f"\nERROR: {data.get('content', 'Unknown')}")
                print("-" * 80)
                print()
        
        except:
            print(f"[{ts}] Response: {msg.data}")
            print()


def main(args=None):
    print()
    print("Starting Grace Debugger...")
    print()
    
    rclpy.init(args=args)
    debugger = GraceDebugger()
    
    try:
        rclpy.spin(debugger)
    except KeyboardInterrupt:
        print("\n\n" + "=" * 80)
        print("DEBUGGER STOPPED")
        print("=" * 80)
        print(f"\nSession summary:")
        print(f"  Text messages: {debugger.text_count}")
        print(f"  Image messages: {debugger.image_count}")
        print(f"  Responses: {debugger.response_count}")
        print()
    finally:
        debugger.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
