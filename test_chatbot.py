#!/usr/bin/env python3
"""
Grace System Diagnostic Tool
Checks all components to identify what's broken
"""

import os
import sys
import requests
import json
from pathlib import Path

print("=" * 70)
print("GRACE SYSTEM DIAGNOSTIC")
print("=" * 70)
print()

# ═══════════════════════════════════════════════════════
# 1. CHECK PYTHON DEPENDENCIES
# ═══════════════════════════════════════════════════════

print("1. CHECKING PYTHON DEPENDENCIES")
print("-" * 70)

dependencies = {
    'rclpy': False,
    'dotenv': False,
    'slack_sdk': False,
    'requests': False
}

for package in dependencies.keys():
    try:
        if package == 'dotenv':
            from dotenv import load_dotenv
            dependencies[package] = True
            print(f"   ✅ python-dotenv installed")
        elif package == 'slack_sdk':
            from slack_sdk import WebClient
            dependencies[package] = True
            print(f"   ✅ slack-sdk installed")
        elif package == 'rclpy':
            import rclpy
            dependencies[package] = True
            print(f"   ✅ rclpy installed")
        elif package == 'requests':
            import requests
            dependencies[package] = True
            print(f"   ✅ requests installed")
    except ImportError:
        print(f"   ❌ {package} NOT installed")
        if package == 'dotenv':
            print(f"      Install: pip3 install python-dotenv")
        elif package == 'slack_sdk':
            print(f"      Install: pip3 install slack-sdk")

print()

# ═══════════════════════════════════════════════════════
# 2. CHECK ENVIRONMENT VARIABLES
# ═══════════════════════════════════════════════════════

print("2. CHECKING ENVIRONMENT VARIABLES")
print("-" * 70)

# Try to load .env
if dependencies['dotenv']:
    from dotenv import load_dotenv
    load_dotenv()
    print("   ✅ Loaded .env file (if exists)")
else:
    print("   ⚠️  python-dotenv not available")

# Check Slack token
slack_token = os.getenv('SLACK_BOT_TOKEN', '')
if not slack_token:
    print("   ❌ SLACK_BOT_TOKEN not set")
    print("      Create .env file with: SLACK_BOT_TOKEN=xoxb-your-token")
elif slack_token.startswith("xoxb-your"):
    print("   ⚠️  SLACK_BOT_TOKEN is placeholder")
    print("      Update with real token from https://api.slack.com/apps")
else:
    print(f"   ✅ SLACK_BOT_TOKEN found: {slack_token[:15]}...{slack_token[-4:]}")

# Check Slack channel
slack_channel = os.getenv('SLACK_CHANNEL', '#grace-logs')
print(f"   ✅ SLACK_CHANNEL: {slack_channel}")

print()

# ═══════════════════════════════════════════════════════
# 3. CHECK OLLAMA
# ═══════════════════════════════════════════════════════

print("3. CHECKING OLLAMA")
print("-" * 70)

ollama_url = "http://localhost:11434"

try:
    response = requests.get(f"{ollama_url}/api/tags", timeout=5)
    if response.status_code == 200:
        print("   ✅ Ollama is running")
        
        models = response.json().get('models', [])
        print(f"   ✅ Found {len(models)} models:")
        
        target_model = "huihui_ai/qwen3-vl-abliterated:4b-instruct-q4_K_M"
        model_found = False
        
        for model in models:
            name = model.get('name', 'unknown')
            size = model.get('size', 0) / (1024**3)  # Convert to GB
            print(f"      - {name} ({size:.1f}GB)")
            
            if target_model in name:
                model_found = True
        
        print()
        if model_found:
            print(f"   ✅ Target model found: {target_model}")
        else:
            print(f"   ⚠️  Target model NOT found: {target_model}")
            print(f"      You need to pull it: ollama pull {target_model}")
    else:
        print(f"   ❌ Ollama returned status {response.status_code}")
        
except requests.exceptions.ConnectionError:
    print("   ❌ Cannot connect to Ollama")
    print("      Is Ollama running? Check: curl http://localhost:11434")
except Exception as e:
    print(f"   ❌ Ollama check failed: {e}")

print()

# ═══════════════════════════════════════════════════════
# 4. CHECK SEARXNG (WEB SEARCH)
# ═══════════════════════════════════════════════════════

print("4. CHECKING SEARXNG (Web Search)")
print("-" * 70)

searxng_url = "http://127.0.0.1:8080"

try:
    response = requests.get(searxng_url, timeout=5)
    if response.status_code in [200, 403]:
        print(f"   ✅ SearXNG is running (status {response.status_code})")
        
        # Try a test search
        try:
            search_response = requests.get(
                f"{searxng_url}/search",
                params={"q": "test", "format": "json"},
                timeout=5
            )
            if search_response.status_code == 200:
                results = search_response.json().get('results', [])
                print(f"   ✅ Web search working ({len(results)} results for 'test')")
            else:
                print(f"   ⚠️  Search returned status {search_response.status_code}")
        except Exception as e:
            print(f"   ⚠️  Search test failed: {e}")
    else:
        print(f"   ⚠️  SearXNG returned status {response.status_code}")
        
except requests.exceptions.ConnectionError:
    print("   ❌ Cannot connect to SearXNG")
    print("      Is SearXNG running? Check: curl http://127.0.0.1:8080")
    print("      Web search will be disabled")
except Exception as e:
    print(f"   ❌ SearXNG check failed: {e}")

print()

# ═══════════════════════════════════════════════════════
# 5. CHECK ROS2
# ═══════════════════════════════════════════════════════

print("5. CHECKING ROS2")
print("-" * 70)

if dependencies['rclpy']:
    try:
        import subprocess
        result = subprocess.run(['ros2', 'topic', 'list'], 
                              capture_output=True, 
                              text=True, 
                              timeout=5)
        
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            print(f"   ✅ ROS2 is working ({len(topics)} topics)")
            
            # Check for Grace's topics
            grace_topics = {
                '/cns/neural_input': False,
                '/cns/image_input': False,
                '/gce/response': False
            }
            
            for topic in topics:
                if topic in grace_topics:
                    grace_topics[topic] = True
            
            print()
            print("   Grace topics:")
            for topic, found in grace_topics.items():
                if found:
                    print(f"      ✅ {topic}")
                else:
                    print(f"      ❌ {topic} (Grace not running)")
        else:
            print("   ❌ ROS2 command failed")
            print(f"      Error: {result.stderr}")
            
    except FileNotFoundError:
        print("   ❌ ros2 command not found")
        print("      Is ROS2 installed and sourced?")
        print("      Run: source /opt/ros/humble/setup.bash")
    except Exception as e:
        print(f"   ❌ ROS2 check failed: {e}")
else:
    print("   ⚠️  rclpy not installed, skipping ROS2 check")

print()

# ═══════════════════════════════════════════════════════
# 6. CHECK ROSBRIDGE (for Web Interface)
# ═══════════════════════════════════════════════════════

print("6. CHECKING ROSBRIDGE (Web Interface Connection)")
print("-" * 70)

try:
    import subprocess
    result = subprocess.run(['ros2', 'node', 'list'], 
                          capture_output=True, 
                          text=True, 
                          timeout=5)
    
    if result.returncode == 0:
        nodes = result.stdout.strip().split('\n')
        
        rosbridge_found = any('rosbridge' in node for node in nodes)
        
        if rosbridge_found:
            print("   ✅ ROSbridge is running")
            print("      Web interface should be able to connect")
        else:
            print("   ❌ ROSbridge NOT running")
            print("      Start it: ros2 launch rosbridge_server rosbridge_websocket_launch.xml")
            print("      Without ROSbridge, web interface cannot connect!")
    else:
        print("   ⚠️  Cannot check ROSbridge")
        
except Exception as e:
    print(f"   ⚠️  ROSbridge check failed: {e}")

print()

# ═══════════════════════════════════════════════════════
# 7. CHECK SLACK CONNECTION
# ═══════════════════════════════════════════════════════

print("7. CHECKING SLACK CONNECTION")
print("-" * 70)

if dependencies['slack_sdk'] and slack_token and not slack_token.startswith("xoxb-your"):
    try:
        from slack_sdk import WebClient
        from slack_sdk.errors import SlackApiError
        
        client = WebClient(token=slack_token)
        
        # Test authentication
        response = client.auth_test()
        print(f"   ✅ Slack authenticated")
        print(f"      Bot: {response['user']}")
        print(f"      Team: {response['team']}")
        
        # Try to send test message
        try:
            test_response = client.chat_postMessage(
                channel=slack_channel,
                text="🧪 Grace diagnostic test - Slack is working!"
            )
            if test_response['ok']:
                print(f"   ✅ Test message sent to {slack_channel}")
            else:
                print(f"   ⚠️  Message send returned not OK")
        except SlackApiError as e:
            error = e.response['error']
            print(f"   ❌ Message send failed: {error}")
            
            if error == 'channel_not_found':
                print(f"      Channel {slack_channel} not found")
            elif error == 'not_in_channel':
                print(f"      Bot not in channel {slack_channel}")
                print(f"      In Slack, type: /invite @{response['user']}")
            elif error == 'missing_scope':
                print(f"      Bot missing permissions")
                print(f"      Add 'chat:write' scope at https://api.slack.com/apps")
                
    except SlackApiError as e:
        print(f"   ❌ Slack API error: {e.response['error']}")
    except Exception as e:
        print(f"   ❌ Slack check failed: {e}")
else:
    if not dependencies['slack_sdk']:
        print("   ⚠️  slack-sdk not installed")
    elif not slack_token:
        print("   ⚠️  SLACK_BOT_TOKEN not set")
    else:
        print("   ⚠️  Slack token is placeholder")

print()

# ═══════════════════════════════════════════════════════
# 8. CHECK FILE PERMISSIONS
# ═══════════════════════════════════════════════════════

print("8. CHECKING FILE SYSTEM")
print("-" * 70)

home = Path.home()

# Check if we can write to home directory
try:
    test_file = home / '.grace_test'
    test_file.write_text('test')
    test_file.unlink()
    print("   ✅ Can write to home directory")
except Exception as e:
    print(f"   ❌ Cannot write to home: {e}")

# Check for existing Grace files
grace_files = {
    '.chat_history.json': home / '.chat_history.json',
    '.daily_reflections.json': home / '.daily_reflections.json',
    '.env': Path('.env')
}

for name, path in grace_files.items():
    if path.exists():
        size = path.stat().st_size
        print(f"   ✅ {name} exists ({size} bytes)")
    else:
        print(f"   ⚠️  {name} not found (will be created)")

print()

# ═══════════════════════════════════════════════════════
# 9. SUMMARY
# ═══════════════════════════════════════════════════════

print("=" * 70)
print("DIAGNOSTIC SUMMARY")
print("=" * 70)

issues = []

if not dependencies['rclpy']:
    issues.append("❌ rclpy not installed (pip3 install rclpy)")

if not dependencies['dotenv']:
    issues.append("⚠️  python-dotenv not installed (pip3 install python-dotenv)")

if not dependencies['slack_sdk']:
    issues.append("⚠️  slack-sdk not installed (pip3 install slack-sdk)")

if not slack_token or slack_token.startswith("xoxb-your"):
    issues.append("❌ SLACK_BOT_TOKEN not configured")

# Check Ollama
try:
    response = requests.get(f"{ollama_url}/api/tags", timeout=2)
    if response.status_code != 200:
        issues.append("❌ Ollama not running")
except:
    issues.append("❌ Ollama not running")

# Check SearXNG
try:
    response = requests.get(searxng_url, timeout=2)
    if response.status_code not in [200, 403]:
        issues.append("⚠️  SearXNG not running (web search disabled)")
except:
    issues.append("⚠️  SearXNG not running (web search disabled)")

if issues:
    print()
    print("ISSUES FOUND:")
    for issue in issues:
        print(f"  {issue}")
    print()
    print("Fix these issues before running Grace!")
else:
    print()
    print("✅ ALL SYSTEMS OPERATIONAL!")
    print()
    print("Grace should be ready to run:")
    print("  1. Terminal 1: ros2 launch rosbridge_server rosbridge_websocket_launch.xml")
    print("  2. Terminal 2: python3 grace_final.py")
    print("  3. Terminal 3: python3 flask_server.py")
    print("  4. Browser: http://localhost:5000")

print("=" * 70)