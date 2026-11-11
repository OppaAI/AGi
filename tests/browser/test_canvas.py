"""Real-browser coverage of retained artwork geometry and persistence."""
import json
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


def test_study_layers_export_import_and_undo(canvas_page, tmp_path):
    page = canvas_page
    errors = []
    page.on('pageerror', lambda error: errors.append(str(error)))
    page.click('#loadStudy')
    page.wait_for_function("document.querySelector('#objectCount').textContent === '56 / 100'")
    assert page.locator('#emptyCanvasState').is_hidden()
    assert page.locator('#layerList button').count() == 56
    # Painting data can be reopened and edited without flattening the curves.
    original = page.evaluate("JSON.parse(localStorage.getItem('grace-canvas-v1'))")
    assert any(item['kind'] == 'curve' for item in original)
    page.locator('#layerList button').first.click()
    page.fill('#fillColor', '#ff0077')
    page.click('#applyStyle')
    assert page.evaluate("JSON.parse(localStorage.getItem('grace-canvas-v1')).at(-1).fill_color") == '#ff0077'
    page.click('#drawUndo')
    assert page.evaluate("JSON.parse(localStorage.getItem('grace-canvas-v1'))") == original
    with page.expect_download() as download_info:
        page.click('#saveProject')
    target = tmp_path / 'artwork.json'
    download_info.value.save_as(target)
    document = json.loads(target.read_text())
    assert document == {'version': 1, 'objects': original}
    page.click('#drawClear')
    page.set_input_files('#importProject', target)
    page.wait_for_function("document.querySelector('#objectCount').textContent === '56 / 100'")
    with page.expect_download() as download_info:
        page.click('#drawExport')
    png = tmp_path / 'artwork.png'
    download_info.value.save_as(png)
    assert png.read_bytes().startswith(b'\x89PNG')
    assert not errors


@pytest.mark.parametrize('width,height', [(1440, 900), (1024, 768), (390, 844)])
def test_workspace_fits_and_keeps_artboard_proportions(canvas_page, width, height):
    page = canvas_page
    page.set_viewport_size({'width': width, 'height': height})
    assert page.evaluate('document.documentElement.scrollWidth') <= width
    box = page.locator('#drawingCanvas').bounding_box()
    assert box['width'] / box['height'] == pytest.approx(1000 / 650, abs=.01)
    assert box['width'] > min(290, width * .6)
    if width > 900:
        prompt = page.locator('#drawPrompt').bounding_box()
        assert prompt['y'] + prompt['height'] < height
    page.click('#themeToggle')
    assert page.locator('html').get_attribute('data-theme') == 'light'


def test_curves_and_gradient_stops_survive_browser_roundtrip(canvas_page):
    page = canvas_page
    art = {'version': 1, 'objects': [{
        'kind': 'curve', 'id': 'petal', 'points': [[100, 350], [100, 0], [600, 0], [600, 350],
                                                 [600, 600], [100, 600], [100, 350]],
        'fill': True, 'stroke': False, 'gradient': 'linear', 'gradient_angle': 90,
        'gradient_stops': [{'offset': 0, 'color': '#ff0000'}, {'offset': 1, 'color': '#0000ff'}]
    }]}
    page.set_input_files('#importProject', {'name': 'curve.json', 'mimeType': 'application/json',
                                          'buffer': json.dumps(art).encode()})
    page.wait_for_function("document.querySelector('#objectCount').textContent === '1 / 100'")
    pixels = page.evaluate('''() => {
      const ctx=document.querySelector('canvas').getContext('2d');
      return [Array.from(ctx.getImageData(350,150,1,1).data), Array.from(ctx.getImageData(350,450,1,1).data)];
    }''')
    assert pixels[0][0] > pixels[0][2]
    assert pixels[1][2] > pixels[1][0]
    page.reload()
    saved = page.evaluate("JSON.parse(localStorage.getItem('grace-canvas-v1'))")
    assert saved[0]['gradient_stops'] == art['objects'][0]['gradient_stops']


def test_grace_playback_locks_tools_and_keeps_editable_layers(canvas_page):
    page = canvas_page
    page.add_init_script('''
      window.ROSLIB = {
        Ros: class { on(name,cb) { if(name==='connection')setTimeout(cb,0); } close() {} },
        Topic: class {
          constructor(opts){this.name=opts.name;}
          subscribe(cb){window.receiveDrawing=cb;}
          publish(message){window.sentDrawing=JSON.parse(message.data);}
        },
        Message: class {constructor(value){Object.assign(this,value);}}
      };
    ''')
    page.reload()
    page.emulate_media(reduced_motion='reduce')
    page.fill('#drawPrompt', 'Paint a rose petal')
    page.click('#drawAsk')
    assert page.locator('#drawClear').is_disabled()
    assert page.locator('#drawGradient').is_disabled()
    assert page.locator('#drawTool').is_disabled()
    page.evaluate('''() => {
      const identifier=window.sentDrawing.identifier;
      const send=event=>window.receiveDrawing({data:JSON.stringify({...event,identifier})});
      send({type:'canvas_reply',content:'A soft coral petal.'});
      send({type:'canvas_actions',actions:[{kind:'curve',id:'rose-petal',
        points:[[200,400],[180,100],[600,100],[550,400],[500,550],[250,550],[200,400]],
        fill:true,stroke:false,fill_color:'#eaa2ac',gradient_color:'#b64b76',gradient:'radial'}]});
      send({type:'canvas_done'});
    }''')
    page.wait_for_function("document.querySelector('#drawStatus').textContent.startsWith('Grace finished')")
    assert page.locator('#drawResponse').inner_text() == 'A soft coral petal.'
    assert page.locator('#layerList button').count() == 1
    assert not page.locator('#drawTool').is_disabled()
    assert page.locator('#drawStop').is_disabled()
