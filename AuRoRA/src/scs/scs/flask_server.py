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
CSS_DIR = BASE_DIR / "css"
JS_DIR = BASE_DIR / "js"

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
                <p>The file <code>index.html</code> was not found.</p>
                <p>Expected location: <code>{WEB_INTERFACE_FILE}</code></p>
                <p>Please make sure the HTML file is in the same directory as this Flask server.</p>
                <hr>
                <h2>Quick Fix:</h2>
                <ol>
                    <li>Make sure <code>index.html</code> is in: <code>{BASE_DIR}</code></li>
                    <li>Restart this server: <code>python3 flask_server.py</code></li>
                </ol>
            </body>
        </html>
        """, 404

@app.route('/css/<path:filename>')
def serve_css(filename):
    """Serve CSS files from css/ directory"""
    if CSS_DIR.exists():
        return send_from_directory(CSS_DIR, filename)
    else:
        return f"CSS directory not found: {CSS_DIR}", 404

@app.route('/js/<path:filename>')
def serve_js(filename):
    """Serve JavaScript files from js/ directory"""
    if JS_DIR.exists():
        return send_from_directory(JS_DIR, filename)
    else:
        return f"JS directory not found: {JS_DIR}", 404

@app.route('/health')
def health():
    """Health check endpoint"""
    return {
        "status": "ok",
        "service": "Grace Web Interface Server",
        "interface_file_exists": WEB_INTERFACE_FILE.exists(),
        "css_dir_exists": CSS_DIR.exists(),
        "js_dir_exists": JS_DIR.exists()
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
        print(f"   Please place index.html in {BASE_DIR}")
    
    # Check for css/ and js/ directories
    if CSS_DIR.exists():
        css_files = list(CSS_DIR.glob('*.css'))
        print(f"✅ Found css/ directory with {len(css_files)} files")
    else:
        print(f"⚠️  WARNING: css/ directory not found!")
        print(f"   Expected: {CSS_DIR}")
    
    if JS_DIR.exists():
        js_files = list(JS_DIR.glob('*.js'))
        print(f"✅ Found js/ directory with {len(js_files)} files")
    else:
        print(f"⚠️  WARNING: js/ directory not found!")
        print(f"   Expected: {JS_DIR}")
    
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
    # debug=True for development
    app.run(
        host='0.0.0.0',
        port=9413,
        debug=True,
        threaded=True
    )