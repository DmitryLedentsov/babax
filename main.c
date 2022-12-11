#include <SDL2/SDL.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "Babax.h"

enum render_constants{
    SCREEN_WIDTH = 640,
    SCREEN_HEIGHT = 480,
};


/*
void render_body(SDL_Renderer* render, struct body* body){
    SDL_SetRenderDrawColor(render,255,0,0,0);
}*/
void render_body(SDL_Renderer* render, struct body* body){
    int color = body->intersect? 255:200;
    printf("%d", body->intersect);
    SDL_SetRenderDrawColor(render,color,0,0,0);
    int n = body->shape.n;
    struct vec* transformed = body->shape.transformed_vertices;
    struct vec rend [MAX_VERTICES];

    SDL_RenderDrawPointF(render, body->position.x, body->position.y);
    //struct vec* p = body.shape.transformed_vertices;
    float x,y;
    for(int i=0; i<n;i++){ 
        struct vec* p = &transformed[i];
        
        x = p->x;
        y = p->y;
        //if(i==0) printf("%f %f \n", x, y);
        rend[i] = *p;

        //SDL_FillRect(screen_surface, &rect, SDL_MapRGB( screen_surface->format, 255, 0, 0));
    }
    transformed = rend;
    SDL_RenderDrawLinesF( render, (const SDL_FPoint*) rend, n );
    SDL_RenderDrawLineF(render, x,y, transformed[0].x, transformed[0].y);
}
void render_all(SDL_Renderer* render, struct body** bodies, int N){
   
    SDL_SetRenderDrawColor(render,0,0,0,0);
    SDL_RenderClear(render);
    SDL_SetRenderDrawColor(render,255,0,0,0);
    for(int i=0;i<N;i++){
        render_body(render, bodies[i]);
    }
	SDL_RenderPresent(render); 
}


int GetRandomValue(int min, int max){
    return (rand() % (max - min + 1)) + min;
}
void mousePress(PhysicsState state,SDL_MouseButtonEvent b){
  if(b.button == SDL_BUTTON_LEFT){
    PhysicsBody bdy = CreatePhysicsBodyPolygon(state,(Vector2){b.x,b.y}, GetRandomValue(20, 80), GetRandomValue(3, 8), 10);
    
  }
}
int main (int argc, char ** args) {
    PhysicsStateData sd =  (PhysicsStateData){ 0 };
    PhysicsState state = &sd;
    InitPhysics(state);

    // Create floor rectangle physics body
    PhysicsBody floor = CreatePhysicsBodyRectangle(state,(Vector2){ SCREEN_WIDTH/2, SCREEN_HEIGHT }, 500, 100, 10);
    floor->enabled = false; // Disable body state to convert it to static (no dynamics, but collisions)

    // Create obstacle circle physics body
    PhysicsBody circle = CreatePhysicsBodyCircle(state,(Vector2){ SCREEN_WIDTH/2, SCREEN_HEIGHT/2 }, 45, 10);
    circle->enabled = true; // Disable body state to convert it to static (no dynamics, but collisions)
    SetPhysicsGravity(state, 0,9);
    //SetPhysicsTimeStep(0.1);

    SDL_Surface* screen_surface = NULL;

    SDL_Window* window = NULL;
    SDL_Renderer *render = NULL;

    window = SDL_CreateWindow("Hello, SDL 2!",SDL_WINDOWPOS_UNDEFINED, 
    SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, 
    SDL_WINDOW_SHOWN);
    
    if (window == NULL) {
        return 1;
    }

    //screen_surface = SDL_GetWindowSurface(window);
    render = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    //SDL_FillRect(screen_surface, NULL, SDL_MapRGB( screen_surface->format, 0, 255, 0));

    


    while (1)
    {
      // Get the next event
      SDL_Event event;
      if (SDL_PollEvent(&event))
      {
    
        if (event.type == SDL_QUIT)
        {
          // Break out of the loop on quit
          break;
        }

        if(event.type == SDL_MOUSEBUTTONDOWN){
                //do whatever you want to do after a mouse button was pressed,
                // e.g.:
                mousePress(state, event.button);
                
        }
      }

        PhysicsStep(state);
        int bodiesCount = GetPhysicsBodiesCount(state);
        for (int i = bodiesCount - 1; i >= 0; i--)
        {
            PhysicsBody body = GetPhysicsBody(state,i);

            if ((body != NULL) && (body->position.y > SCREEN_HEIGHT*2))
                DestroyPhysicsBody(state,body);
        }


        SDL_SetRenderDrawColor(render,0,0,0,0);
        SDL_RenderClear(render);
        SDL_SetRenderDrawColor(render,255,0,0,0);
        bodiesCount = GetPhysicsBodiesCount(state);
        
        for (int i = 0; i < bodiesCount; i++)
        {
            PhysicsBody body = GetPhysicsBody(state, i);
            if (body != NULL)
            {
                int vertexCount = GetPhysicsShapeVerticesCount(state, i);
                for (int j = 0; j < vertexCount; j++)
                {
                    // Get physics bodies shape vertices to draw lines
                    // Note: GetPhysicsShapeVertex() already calculates rotation transformations
                    Vector2 vertexA = GetPhysicsShapeVertex(body, j);
                    int jj = (((j + 1) < vertexCount) ? (j + 1) : 0);   // Get next vertex or first to close the shape
                    Vector2 vertexB = GetPhysicsShapeVertex(body, jj);
                    SDL_RenderDrawLineF(render,vertexA.x, vertexA.y, vertexB.x, vertexB.y);     // Draw a line between two vertex positions
                }
            }
        }
        SDL_RenderPresent(render); 
        SDL_Delay(1000/SDL_GetTicks());
      /*
       update_all(bodies,N);

        render_all(render, bodies,N);*/
    }
    
    
    //SDL_Delay(2000);
    ClosePhysics(state);
    SDL_DestroyWindow(window);
    SDL_DestroyRenderer(render);
    SDL_Quit();

    return 0;
};