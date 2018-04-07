from flask import Flask
from flask import request, render_template
from path_planning_model import drone_path_planning_model
import requests
import os




app = Flask(__name__)


# http://35.185.197.49:5000/?date=2018-03-16&n=200
@app.route('/')
def CE290I_API():
    # here we want to get the value of user (i.e. ?user=some-value)
    date = request.args.get('date')
    data_n = int(request.args.get('n'))
    r = requests.get('https://ce290-hw5-weather-report.appspot.com/', params={'date': date})
    attributes = r.json()
    model = drone_path_planning_model(n=data_n, centroid_x=attributes['centroid_x'], centroid_y=attributes['centroid_y'], radius=attributes['radius'])
    model.planning()
    img_path = 'static/path.png'
    model.save_figure(img_path)
    return render_template('index.html', img_path = img_path, shortest_dist = model.shortest_distance)

if __name__ == '__main__':
    app.run()