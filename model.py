import pandas as pd  # To manage data as data frames
import numpy as np  # To manipulate data as arrays
from sklearn.linear_model import LogisticRegression

df = pd.read_csv('./Dataset1.csv')

print(df.head())

df['Temp'] = pd.to_numeric(df.Temp, errors='coerce')
df.fillna(df.mean(), inplace=True)

s1 = 1/df['ph']
print(s1)

s2 = 1/df['TDS']
print(s2)

s3 = 1/df['Turbidity']
print(s3)

s4 = 1/df['Conductivity']
print(s4)

x = s1+s2+s3+s4
print(x)

k = 1/x
print(k)

w1 = k/df['ph']
print(w1)

w2 = k/df['Temp']
print(w2)

w3 = k/df['Turbidity']
print(w3)

w4 = k/df['Conductivity']
print(w4)

v1 = df['ph'].mean()
print(v1)

v2 = df['Temp'].mean()
print(v2)

v3 = df['Turbidity'].mean()
print(v3)

v4 = df['Conductivity'].mean()
print(v4)

q1 = v1*100/df['ph']
print(q1)

q2 = v2*100/df['Temp']
print(q2)

q3 = v3*100/df['Turbidity']
print(q3)

q4 = v4*100/df['Conductivity']
print(q4)

z1 = w1*q1
print(z1)

z2 = w2*q2
print(z2)

z3 = w3*q3
print(z3)

z4 = w4*q4
print(z4)

WQI = z1+z2+z3+z4
print(WQI)

X = df.iloc[:, 0:-1]
y = df.iloc[:, -1]
# print("----------")
# print(X)
# print("----------")
# print(y)
# print("----------")


# Initializing the Logistic Regression model
logreg = LogisticRegression(max_iter=1000)
logreg.fit(X, y)  # Fitting the model

data = df.replace([0, 1], ['Not Potable', 'Potable'])

# Dictionary containing the mapping
variety_mappings = {0: 'Not Potable', 1: 'Potable'}

# Encoding the target variables to integers
new_data = data.replace(['Not Potable', 'Potable'], [0, 1])


def classify(a, b, c, d, e):
    arr = np.array([a, b, c, d, e])  # Convert to numpy array
    arr = arr.astype(np.float64)  # Change the data type to float
    query = arr.reshape(1, -1)  # Reshape the array
    prediction = variety_mappings[logreg.predict(
        query)[0]]  # Retrieve from dictionary
    return prediction  # Return the prediction
