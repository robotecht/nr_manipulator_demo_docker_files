from flask import Flask, render_template, request, redirect
import subprocess
import os
from subprocess import Popen, PIPE
from subprocess import check_output


def pick_place_sim():
    os.environ['RUN_MODE'] = "sim"
    subprocess.call("/home/nr-ws-1/Desktop/nr_manipulator_demo_docker_files/demo_pick_and_place/run.sh", shell=True)
def pick_place_sim_real():
    os.environ['RUN_MODE'] = "hardware"
    subprocess.call("/home/nr-ws-1/Desktop/nr_manipulator_demo_docker_files/demo_pick_and_place/run.sh", shell=True)

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
    if request.method == 'POST':
        pick_place_sim()
        return redirect('/thankyou.html')
    else:
        return 'something went wrong. Try again!'


@app.route('/start_demo_real',methods=['POST','GET'])
def real():
    if request.method == 'POST':
        pick_place_sim_real()
        return redirect('/thankyou.html')
    else:
        return 'something went wrong. Try again!'

@app.route('/submit_form',methods=['POST','GET'])
def submit_form():
    if request.method == 'POST':
        data = request.form.to_dict()
        write_to_file(data)
        return redirect('/thankyou.html')
    else:
        return 'something went wrong. Try again!'
    

if __name__=="__main__":
    app.run(host='0.0.0.0',port=4444, debug=True)