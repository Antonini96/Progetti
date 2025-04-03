"""
Questo codice esegue un loop in cui gioca a Tetris per 5000 passi 
utilizzando azioni casuali e mostra il gioco in esecuzione. Nota che l'agente 
non sta imparando o ottimizzando alcuna strategia, ma sta semplicemente agendo 
casualmente per dimostrazione o scopi di debugging.
"""

#libreria per l'emulazione di giochi Nintendo Entertainment System (NES) e fornisce funzionalità per collegare un agente di apprendimento automatico agli emulatori NES
from nes_py.wrappers import JoypadSpace
#Importa l'ambiente personalizzato Gym chiamato gym_tetris, che permette di utilizzare l'emulatore NES per il gioco Tetris
import gym_tetris
#Importa l'elenco delle azioni disponibili per l'ambiente Tetris
from gym_tetris.actions import MOVEMENT

#Crea l'ambiente di gioco Tetris utilizzando la variante specifica 'TetrisA-v0'
env = gym_tetris.make('TetrisA-v0') 
#Incapsula l'ambiente Tetris all'interno di JoypadSpace, consentendo l'interazione con l'agente tramite le azioni definite nel set MOVEMENT
env = JoypadSpace(env, MOVEMENT)

done = True #episodio terminato
selected_action_index = 2

for step in range(2):
    
    
    if done:
        state = env.reset() #reinizializza lo stato dell'ambiente chiamando e e ottiene il nuovo stato iniziale
        done = False
    #Esegue un passo nell'ambiente utilizzando un'azione casuale presa dallo spazio delle azioni disponibili
    
    while not done: 
        
        #state, reward, done, info = env.step(env.action_space.sample())
        state, reward, done, info = env.step(env.action_space.sample())
        #state, reward, done, info = env.step(2)
        env.render()

    
    #state, reward, done, info = env.step(env.action_space.sample())
    
    #Visualizza il rendering dell'ambiente, mostrando il gioco Tetris in esecuzione
    env.render()

#Chiude l'ambiente di gioco Tetris e rilascia le risorse associate
env.close()

#info ha le specifiche dello stato del tetris, per guardare il valore di queste 
#specifiche fare ad esempio info['score']