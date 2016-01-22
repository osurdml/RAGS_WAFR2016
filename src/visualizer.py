import sys, random, math, pygame
import numpy as np
import matplotlib.pyplot as plt
from pygame.locals import *
from sklearn.preprocessing import normalize


#constants
XDIM = 640
YDIM = 480
WINSIZE = [XDIM, YDIM]
scale = 4
offset = 1

def main():
        vertices = np.genfromtxt('../results/vertices6.txt', delimiter=',')
        edges = np.genfromtxt('../results/edges6.txt', delimiter=',')
        variances = edges[:,3]
        ragsPath = np.genfromtxt('../results/ragsPath6.txt', delimiter=',')
        optimalPath = np.genfromtxt('../results/optimal1.txt', delimiter=',')
        vertices = (vertices.astype(int)+offset)*scale
        edges = edges.astype(int)
        ragsPath = (ragsPath.astype(int)+offset)*scale
        optimalPath = (optimalPath.astype(int)+offset)*scale
        #initialize and prepare screen
        pygame.init()
        screen = pygame.display.set_mode(WINSIZE)
        white = 255, 255, 255
        black = 20, 20, 40
        red = 255, 0, 0
        green = 0, 255, 0
        screen.fill(white)
        norm1 = variances/np.linalg.norm(variances)

        for i in range(0,len(edges[:,0])):
            #print vertices[i,:], vertices[edges[i,j],:]
            norm1 = (variances[i]-min(variances))/(max(variances)-min(variances))
            v = (norm1*200).astype(int)
            #thickness = (norm1*3).astype(int)
            pygame.draw.line(screen,[v,v,v],(vertices[edges[i,0],0], vertices[edges[i,0],1]), (vertices[edges[i,1],0], vertices[edges[i,1],1]), 1)
            pygame.display.update()
        raw_input("Press Enter to continue...")
        for i in range(0,len(optimalPath[:,0])-1):
            pygame.draw.line(screen,green,(optimalPath[i,0], optimalPath[i,1]), (optimalPath[i+1,0],optimalPath[i+1,1]), 2)
            pygame.display.update()
        raw_input("Press Enter to continue...")
        pygame.draw.line(screen,red,((0+offset)*scale, (0+offset)*scale), (ragsPath[0,0], ragsPath[0,1]), 2)
        for i in range(0,len(ragsPath[:,0])-1):
            if(ragsPath[i,0] == (100+offset)*scale and ragsPath[i,1] == (100+offset)*scale):
                pygame.draw.line(screen,red,((0+offset)*scale, (0+offset)*scale), (ragsPath[i+1,0], ragsPath[i+1,1]), 2)
                #raw_input("Press Enter to continue...")
            else:
                pygame.draw.line(screen,red,(ragsPath[i,0], ragsPath[i,1]), (ragsPath[i+1,0],ragsPath[i+1,1]), 2)
            pygame.display.update()

        raw_input("Press Enter to END...")

# if python says run, then we should run
if __name__ == '__main__':
        main()


