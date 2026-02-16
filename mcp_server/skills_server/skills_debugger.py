#!/usr/bin/env python3
"""
Proper MCP Client Test for Skills Server
Follows correct MCP protocol with initialization
"""

import subprocess
import json
import time
import sys
import select

def send_request(process, method, params=None, request_id=1):
    """Send JSON-RPC request"""
    request = {
        "jsonrpc": "2.0",
        "id": request_id,
        "method": method
    }
    if params is not None:
        request["params"] = params
    
    process.stdin.write(json.dumps(request) + '\n')
    process.stdin.flush()

def read_response(process, timeout=5):
    """Read JSON-RPC response"""
    if select.select([process.stdout], [], [], timeout)[0]:
        line = process.stdout.readline()
        if line:
            return json.loads(line)
    return None

def test_mcp_server():
    """Test MCP server with proper protocol"""
    print("=" * 60)
    print("MCP Nature Skills Server Test")
    print("=" * 60)
    
    # Start server
    print("\n1. Starting server...")
    server = subprocess.Popen(
        ["python3", "skills_server.py"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1
    )
    
    time.sleep(1)
    
    if server.poll() is not None:
        print("❌ Server failed to start")
        return False
    
    print("✅ Server started (PID: {})".format(server.pid))
    
    # Step 1: Initialize
    print("\n2. Initializing MCP session...")
    send_request(server, "initialize", {
        "protocolVersion": "2024-11-05",
        "capabilities": {
            "roots": {"listChanged": False},
            "sampling": {}
        },
        "clientInfo": {
            "name": "test-client",
            "version": "1.0.0"
        }
    })
    
    response = read_response(server, timeout=5)
    if not response or 'error' in response:
        print(f"❌ Initialize failed: {response}")
        server.terminate()
        return False
    
    print("✅ Session initialized")
    
    # Step 2: Send initialized notification
    print("\n3. Sending initialized notification...")
    notification = {
        "jsonrpc": "2.0",
        "method": "notifications/initialized"
    }
    server.stdin.write(json.dumps(notification) + '\n')
    server.stdin.flush()
    time.sleep(0.5)
    
    # Step 3: List tools
    print("\n4. Listing available tools...")
    send_request(server, "tools/list", {}, request_id=2)
    
    response = read_response(server, timeout=5)
    if not response:
        print("❌ No response to tools/list")
        server.terminate()
        return False
    
    if 'error' in response:
        print(f"❌ Error: {response['error']}")
        server.terminate()
        return False
    
    if 'result' not in response:
        print(f"❌ Unexpected response: {response}")
        server.terminate()
        return False
    
    tools = response['result'].get('tools', [])
    print(f"✅ Found {len(tools)} tools:")
    for tool in tools:
        print(f"   • {tool['name']}")
        print(f"     {tool['description'][:60]}...")
    
    # Step 4: Call a tool
    print("\n5. Testing tool call: get_weather...")
    send_request(server, "tools/call", {
        "name": "get_weather",
        "arguments": {
            "location": "Vancouver",
            "days": 1
        }
    }, request_id=3)
    
    response = read_response(server, timeout=10)
    if not response:
        print("❌ No response to tool call")
        server.terminate()
        return False
    
    if 'error' in response:
        print(f"❌ Tool call error: {response['error']}")
        server.terminate()
        return False
    
    if 'result' in response:
        content = response['result'].get('content', [])
        if content:
            text = content[0].get('text', '')
            print("✅ Weather result:")
            print(text)
        else:
            print("❌ No content in result")
            server.terminate()
            return False
    else:
        print(f"❌ Unexpected response: {response}")
        server.terminate()
        return False
    
    # Cleanup
    print("\n6. Stopping server...")
    server.terminate()
    server.wait(timeout=5)
    print("✅ Server stopped")
    
    print("\n" + "=" * 60)
    print("✅ ALL TESTS PASSED!")
    print("=" * 60)
    return True

if __name__ == "__main__":
    import os
    
    # Check file exists
    if not os.path.exists("skills_server.py"):
        print("❌ skills_server.py not found in current directory")
        print(f"Current directory: {os.getcwd()}")
        sys.exit(1)
    
    try:
        success = test_mcp_server()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n❌ Test interrupted")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n❌ Test failed with exception: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)