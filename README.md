This system uses data collected from various datasets to train the ML model using Logistic Regression. This model is deployed using the Flask framework. It also comprises 5 IoT sensors (ph, tds, temperature, turbidity, conductivity) which collect data in real-time. A webpage is made where any user can see real-time data of water quality parameters in form of graphs (data vs time). Using the deployed model any user can check whether water is potable for use or not by inputting the value fetched from IoT sensors. Also, it will predict what kind of disease you may have if you will continue drinking this water.