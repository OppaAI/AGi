#!/usr/bin/env python3
"""
Flask Web Server for Grace Chat Interface

This serves the Grace web interface and provides a simple HTTP server
for accessing the chat UI through a web browser.

Usage:
    python3 flask_server.py

Then open: http://localhost:9413 or http://<your-ip>:9413
"""

from flask import Flask, send_file, send_from_directory
from pathlib import Path
import os

app = Flask(__name__)

# Get the directory where this script is located
BASE_DIR = Path(__file__).parent.absolute()

# Path to the web interface HTML file
WEB_INTERFACE_FILE = BASE_DIR / "AGi.html"

@app.route('/')
def index():
    """Serve the main Grace chat interface"""
    if WEB_INTERFACE_FILE.exists():
        return send_file(WEB_INTERFACE_FILE)
    else:
        return f"""
        <html>
            <head><title>Error - Grace Not Found</title></head>
            <body style="font-family: Arial; padding: 40px; background: #0a0a0a; color: #00ff88;">
                <h1 style="color: #ff69b4;">❌ Grace Web Interface Not Found</h1>
                <p>The file <code>grace_web_interface_fixed.html</code> was not found.</p>
                <p>Expected location: <code>{WEB_INTERFACE_FILE}</code></p>
                <p>Please make sure the HTML file is in the same directory as this Flask server.</p>
                <hr>
                <h2>Quick Fix:</h2>
                <ol>
                    <li>Make sure <code>grace_web_interface_fixed.html</code> is in: <code>{BASE_DIR}</code></li>
                    <li>Restart this server: <code>python3 flask_server.py</code></li>
                </ol>
            </body>
        </html>
        """, 404

@app.route('/health')
def health():
    """Health check endpoint"""
    return {
        "status": "ok",
        "service": "Grace Web Interface Server",
        "interface_file_exists": WEB_INTERFACE_FILE.exists()
    }

@app.route('/favicon.ico')
def favicon():
    """Prevent 404 errors for favicon requests"""
    return '', 204

if __name__ == '__main__':
    print("=" * 60)
    print("🌸 Grace Web Interface Server Starting...")
    print("=" * 60)
    
    # Check if the interface file exists
    if WEB_INTERFACE_FILE.exists():
        print(f"✅ Found interface file: {WEB_INTERFACE_FILE.name}")
    else:
        print(f"⚠️  WARNING: Interface file not found!")
        print(f"   Expected: {WEB_INTERFACE_FILE}")
        print(f"   Please place AGi.html in {BASE_DIR}")
        print()
    
    print("-" * 60)
    print("📡 Server Configuration:")
    print(f"   Host: 0.0.0.0 (accessible from any device on your network)")
    print(f"   Port: 9413")
    print("-" * 60)
    print("🌐 Access Grace at:")
    print(f"   Local:   http://localhost:9413")
    print(f"   Network: http://<your-ip>:9413")
    print()
    print("💡 To find your IP address:")
    print("   Linux/Mac: ifconfig | grep 'inet '")
    print("   Windows:   ipconfig")
    print("-" * 60)
    print("Press Ctrl+C to stop the server")
    print("=" * 60)
    print()
    
    # Run the Flask server
    # host='0.0.0.0' makes it accessible from other devices on your network
    # debug=False for production, set to True for development
    app.run(
        host='0.0.0.0',
        port=9413,
        debug=False,
        threaded=True
    )
