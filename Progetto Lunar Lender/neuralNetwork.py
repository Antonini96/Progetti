import numpy as np

import pandas as pd #libreria python per l'analisi dei dati

#input da dare alla rete neurale
CSV_URL = "https://raw.githubusercontent.com/ProfAI/tutorials/master/Come%20Creare%20una%20Rete%20Neurale%20da%20Zero/breast_cancer.csv"

breast_cancer = pd.read_csv(CSV_URL)

#estraiamo features e target in array numpy
X = breast_cancer.drop("malignant", axis=1).values
y = breast_cancer["malignant"].values

#creiamo la classe neuralNetwork con un solo strato nascosto
class NeuralNetwork:
  
  
    def __init__(self, hidden_layer_size=100): #hidden_layer_size = numero di nodi
       
        self.hidden_layer_size=hidden_layer_size
       
       
    def _init_weights(self, input_size, hidden_size):
       
        self._W1 = np.random.randn(input_size, hidden_size)
        self._b1 = np.zeros(hidden_size)
        self._W2 = np.random.randn(hidden_size,1)
        self._b2 = np.zeros(1)
   
   
    #METRICHE=misure per definire quanto funziona una rete neurale   
    #misura l'accuratezza della rete neurale
    def _accuracy(self, y, y_pred):      
        return np.sum(y==y_pred)/len(y)
     
    #calcoliamo la Log loss, da affiancare alla accuracy 
    def _log_loss(self, y_true, y_proba):
        return -np.sum(np.multiply(y_true,np.log(y_proba))+np.multiply((1-y_true),np.log(1-y_proba)))/len(y_true)
     
       
    #funzione di attivazione ReLu che serve per elaborare gli output dello strato
    #precedente come input dello stato a cui arrivano 
    def _relu(self, Z):
        return np.maximum(Z, 0)
   
    #come la ReLu, questa è la sigmoide, utilizzata per la classificazione binaria 
    def _sigmoid(self, Z):
        return 1/(1+np.power(np.e,-Z))
     
     
    def _relu_derivative(self, Z):
        dZ = np.zeros(Z.shape)
        dZ[Z>0] = 1
        return dZ
       
    #metodo per la forward propagation              
    def _forward_propagation(self, X):
                        
        Z1 = np.dot(X,self._W1)+self._b1

        A1 = self._relu(Z1)
        Z2 = np.dot(A1,self._W2)+self._b2
        A2 = self._sigmoid(Z2)
       
        self._forward_cache = (Z1, A1, Z2, A2)
   
        return A2.ravel()
   
    #Predizione
    def predict(self, X, return_proba=False):
   
        proba = self._forward_propagation(X)
   
        y = np.zeros(X.shape[0])
        y[proba>=0.5]=1
        y[proba<0.5]=0
   
        if(return_proba):
            return (y, proba)
        else:
            return proba
                               
    #ADDESTRAMENTO
    #Descent gradient
    
    #backpropagation
    def _back_propagation(self, X, y):
     
        Z1, A1, Z2, A2 = self._forward_cache
                      
        m = A1.shape[1]
       
        dZ2 = A2-y.reshape(-1,1)
        dW2 = np.dot(A1.T, dZ2)/m
        db2 = np.sum(dZ2, axis=0)/m
   
        dZ1 = np.dot(dZ2, self._W2.T)*self._relu_derivative(Z1)
        dW1 = np.dot(X.T, dZ1)/m
        db1 = np.sum(dZ1, axis=0)/m
       
        return dW1, db1, dW2, db2
              
    #inizializzazione dei pesi
    def fit(self, X, y, epochs=200, lr=0.01):
        
        self._init_weights(X.shape[1], self.hidden_layer_size)
         
        for _ in range(epochs):
            Y = self._forward_propagation(X)
            dW1, db1, dW2, db2 = self._back_propagation(X, y)
            self._W1-=lr*dW1
            self._b1-=lr*db1
            self._W2-=lr*dW2
            self._b2-=lr*db2
                  
    #esegue le predizioni, calcola le metriche e le ritorna
    def evaluate(self, X, y):
        y_pred, proba = self.predict(X, return_proba=True)
        accuracy = self._accuracy(y, y_pred)
        log_loss = self._log_loss(y, proba)
        return (accuracy, log_loss)    
     
#assegniamo il 30% degli esempi del dataset al set di test, e quindi il 70% al TS
def train_test_split(X, y, test_size=0.3, random_state=None):

    if(random_state!=None):
        np.random.seed(random_state)
 
    n = X.shape[0]

    test_indices = np.random.choice(n, int(n*test_size), replace=False) # selezioniamo gli indici degli esempi per il test set
 
    # estraiamo gli esempi del test set
    # in base agli indici
 
    X_test = X[test_indices]
    y_test = y[test_indices]
 
    # creiamo il train set
    # rimuovendo gli esempi del test set
    # in base agli indici
     
    X_train = np.delete(X, test_indices, axis=0)
    y_train = np.delete(y, test_indices, axis=0)
   
    return (X_train, X_test, y_train, y_test )

X_train, X_test, y_train, y_test  = train_test_split(X, y, test_size=0.3)
    
#normalizzazione delle features
   
X_max = X_train.max(axis=0)
X_min = X_train.min(axis=0)

X_train = (X_train - X_min)/(X_max-X_min)
X_test = (X_test - X_min)/(X_max-X_min)
     
model = NeuralNetwork()
model.fit(X_train, y_train, epochs=500, lr=0.01)
model.evaluate(X_test, y_test)
     
     
#proviamola su 6 nuovi casi
exams_df = pd.read_csv("https://raw.githubusercontent.com/ProfAI/tutorials/master/Come%20Creare%20una%20Rete%20Neurale%20da%20Zero/exam%20results.csv")

X_new = exams_df.values
X_new = (X_new - X_min)/(X_max-X_min)
     
y_pred, y_proba = model.predict(X_new, return_proba=True)
     
classes = ["benigno", "maligno"]

for i, (pred, proba) in enumerate(zip(y_pred, y_proba)):
    print("Risultato %d = %s (%.4f)" % (i+1, classes[int(pred)], proba))


"""
Risultato 1 = benigno (0.0000)
Risultato 2 = maligno (0.9982)
Risultato 3 = maligno (0.9982)
Risultato 4 = benigno (0.0103)
Risultato 5 = maligno (0.6891)
Risultato 6 = benigno (0.0316)
"""