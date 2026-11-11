"""Real-browser coverage of retained artwork geometry and persistence."""
import shutil
import threading
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

playwright = pytest.importorskip('playwright.sync_api')
CONSOLE = Path(__file__).resolve().parents[2] / 'entity/telepathic_management/telepathic_management/telepathic_console'


@pytest.fixture
def canvas_page():
    class Handler(SimpleHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def translate_path(self, path):
            if path.startswith('/static/'):
                return str(CONSOLE / 'backend' / path.lstrip('/'))
            return super().translate_path(path)

    server = ThreadingHTTPServer(('127.0.0.1', 0), partial(Handler, directory=str(CONSOLE / 'frontend')))
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    try:
        with playwright.sync_playwright() as p:
            executable = shutil.which('chromium') or shutil.which('google-chrome')
            with p.chromium.launch(headless=True, executable_path=executable) as browser:
                page = browser.new_page(viewport={'width': 1440, 'height': 1000})
                page.route('https://cdn.jsdelivr.net/**', lambda route: route.abort())
                page.goto(f'http://127.0.0.1:{server.server_port}/canvas.html')
                yield page
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=2)


def drag(page, start, end):
    box = page.locator('#drawingCanvas').bounding_box()
    def pos(point):
        return box['x'] + point[0] / 1000 * box['width'], box['y'] + point[1] / 650 * box['height']
    page.mouse.move(*pos(start))
    page.mouse.down()
    page.mouse.move(*pos(end), steps=4)
    page.mouse.up()


def test_ellipse_pixels_match_retained_bounds_and_move_survives_reload(canvas_page):
    page = canvas_page
    page.select_option('#drawTool', 'ellipse')
    page.check('#drawFill')
    page.uncheck('#drawStroke')
    drag(page, (100, 100), (400, 200))
    bounds = page.evaluate('''() => {
      const data=document.querySelector('#drawingCanvas').getContext('2d').getImageData(0,0,1000,650).data;
      let minX=1000,maxX=0,minY=650,maxY=0;
      for(let y=0;y<650;y++)for(let x=0;x<1000;x++)if(data[(y*1000+x)*4+3]>100){
        minX=Math.min(minX,x);maxX=Math.max(maxX,x);minY=Math.min(minY,y);maxY=Math.max(maxY,y);
      }
      return [minX,minY,maxX,maxY];
    }''')
    assert bounds == pytest.approx([100, 100, 399, 199], abs=3)
    page.select_option('#drawTool', 'select')
    drag(page, (250, 150), (995, 645))
    saved = page.evaluate("JSON.parse(localStorage.getItem('grace-canvas-v1'))")
    assert saved[0]['x'] + saved[0]['width'] <= 1000.01
    assert saved[0]['y'] + saved[0]['height'] <= 650.01
    page.reload()
    assert page.evaluate("JSON.parse(localStorage.getItem('grace-canvas-v1'))") == saved
