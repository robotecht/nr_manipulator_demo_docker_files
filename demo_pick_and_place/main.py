from flask import Flask, render_template, request, redirect
import subprocess
import os
from subprocess import Popen, PIPE
from subprocess import check_output


def pick_place_sim():
    del os.environ['RUN_MODE']
    os.environ['RUN_MODE'] = "sim"
    subprocess.call("run.sh", shell=True)
def pick_place_sim_real():
    del os.environ['RUN_MODE']
    os.environ['RUN_MODE'] = "hardware"
    subprocess.call("run.sh", shell=True)

def write_to_file(data):
    with open('database.txt',mode='a') as database:
        name = data["name"]
        email= data["email"]
        message = data["message"]
        file = database.write(f'\n{name},{email},{message}')


app = Flask(__name__)

@app.route('/')
def run_demo():
    return render_template('index.html')
@app.route('/<string:page_name>')
def html_page(page_name):
    return render_template(page_name)
@app.route('/start_demo_sim',methods=['POST','GET'])
def sim():
    pick_place_sim()
    return 'running sim'

@app.route('/start_demo_real',methods=['POST','GET'])
def real():
    pick_place_sim_real()
    return 'running real'

@app.route('/submit_form',methods=['POST','GET'])
def submit_form():
    if request.method == 'POST':
        data = request.form.to_dict()
        write_to_file(data)
        return redirect('/thankyou.html')
    else:
        return 'something went wrong. Try again!'