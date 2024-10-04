from sklearn.ensemble import RandomForestRegressor
import numpy as np
import pickle

# scaler
MinMaxScaler = pickle.load(open("Eco2.0Accel-1TO1AccelStep025ETEDScaler_20.p", "rb"))
# regressor
nextDesiredVelocityRegressor = pickle.load(open("Eco2.0Accel-1TO1AccelStep025ETEDRegressor_20.p", "rb"))


def predict_velocity_v1(minimumTimeToPass, maximumTimeToPass, initialVelocity, distanceToTravel):
    # syntax sugar
    firstPrediction = 0

    # format parameters for prediction
    numpyArrayFeatures = np.array([minimumTimeToPass,
                                   maximumTimeToPass, initialVelocity, distanceToTravel])
    numpyArrayFeatures = np.expand_dims(numpyArrayFeatures, axis=0)

    # normalize the inputs
    numpyArrayFeatures = MinMaxScaler.transform(numpyArrayFeatures)

    # get the prediction
    predictionFromRegressor = nextDesiredVelocityRegressor.predict(numpyArrayFeatures)[firstPrediction]

    return (predictionFromRegressor)