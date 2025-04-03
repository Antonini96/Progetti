% Main file to perform detection of non-coding RNAs on the basis of predicted 
% secondary structure formation free energy change using a multilayer neural network
%
% Class of "Metodi di Ottimizzazione per Big Data" ("Optimization Methods for Big Data")
% University of Rome "Tor Vergata"
%
% Authors: Antonini Davide 0316030
%          Di Vincenzo Fabio 0310272
%          Menichelli Alberto 0308559
% Date: September 12nd, 2023

clear all
close all
clc

%% punto 1 

% % per creare le matrici numeriche X e Y che separano le features e le
% % label. Per dataset del tipo '-1 1:-766 2:128 3:0.140625 4:0.304688 ...'
% filename='cod-rna_train.txt';
% %import the data as X e Y
% [X, Y] = createMatrix(filename);
%poi le salvo come file .mat e le carico sul programma

%Load the data
[x_train, y_train, x_test, y_test, vettore_labels] = divideMatrix();

%% creazione della rete neurale
layer = NeuralNetwork; 

%% parametri che caratterizzano la rete neurale
firstHiddenLay = 10; %neuroni presenti nel primo strato nascosto
secondHiddenLay = 2; %neuroni presenti nel secondo strato nascosto
layer.epochs = 10; %numero di epoche
layer.lr = 0.001; %learning rate (tasso di apprendimento)
layer.n_sample = 16; %numeri di campioni presi per volta/Batch size

%% inserimento di questi parametri dentro la rete neurale
layer.Features = size(x_train, 1); %numero di neuroni strato di input
layer.Classes = length(vettore_labels); %neuroni nello strato di output
layer.Layer_Neurons = [layer.Features, firstHiddenLay, secondHiddenLay, layer.Classes]; %neuroni di tutti i strati
layer.N_HiddenLayer = size(layer.Layer_Neurons, 2); %numero di strati totali
layer.vettore_labels = vettore_labels; %vettore contenente le etichette presenti

%% risultati del training set e test set
layer = layer.fit(x_train, y_train); %training, addestramento rete neurale
layer = layer.evaluate(x_test, y_test); %test, valutazione rete neurale
