from flask import Flask, render_template
import subprocess
from subprocess import Popen, PIPE
from subprocess import check_output


def pick_place():
    subprocess.call("run.sh", shell=True)

app = Flask(__name__)

@app.route('/')
def run_demo():
    return render_template('index.html')
@app.route('/<string:page_name>')
def html_page(page_name):
    return render_template(page_name)
@app.route('/start_demo',methods=['POST','GET'])
def home():
    pick_place()
    return 'running'