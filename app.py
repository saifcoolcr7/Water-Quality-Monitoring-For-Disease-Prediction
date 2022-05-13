import model  # Import the python file containing the ML model
from flask import Flask, request, render_template

# Initialize the flask class and specify the templates directory
app = Flask(__name__, template_folder="templates")

# Default route set as 'home'


@app.route('/')
def home():
    return render_template('home.html')  # Render home.html


# Route 'classify' accepts GET request
@app.route('/classify', methods=['GET'])
def classify_type():
    try:
        ph = request.args.get('ph_value')
        tds = request.args.get('tds_value')
        turbidity = request.args.get('turbidity_value')
        temperature = request.args.get(
            'temperature_value')
        conductivity = request.args.get(
            'conductivity_value')

        # Get the output from the classification model
        variety = model.classify(ph, tds, turbidity, temperature, conductivity)

        # Render the output in new HTML page
        return render_template('output.html', variety=variety, ph=ph, tds=tds, turbidity=turbidity, conductivity=conductivity)
    except:
        return 'Error'


# Run the Flask server
if(__name__ == '__main__'):
    app.run(debug=True)
