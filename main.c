#include <SDL2/SDL.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "Babax.h"

enum render_constants
{
    SCREEN_WIDTH = 640,
    SCREEN_HEIGHT = 480,
};

/*
void render_body(SDL_Renderer* render, struct body* body){
    SDL_SetRenderDrawColor(render,255,0,0,0);
}*/

int GetRandomValue(int min, int max)
{
    return (rand() % (max - min + 1)) + min;
}
void mousePress(PhysicsState state, SDL_MouseButtonEvent b)
{
    if (b.button == SDL_BUTTON_LEFT)
    {
        PhysicsBody bdy = CreatePhysicsBodyPolygon(state, (Vector2){b.x, b.y}, GetRandomValue(20, 80), GetRandomValue(3, 8), 10);
    }
}
int main(int argc, char **args)
{
    PhysicsStateData sd = (PhysicsStateData){0};
    PhysicsState state = &sd;
    InitPhysics(state);

    // Create floor rectangle physics body
    PhysicsBody floor = CreatePhysicsBodyRectangle(state, (Vector2){SCREEN_WIDTH / 2, SCREEN_HEIGHT}, 500, 100, 10);
    floor->enabled = false; // Disable body state to convert it to static (no dynamics, but collisions)

    /*PhysicsBody c = CreatePhysicsBodyCircle(state,(Vector2){ SCREEN_WIDTH/2, SCREEN_HEIGHT/6 }, 45, 10);
    c->enabled = true; // Disable body state to convert it to static (no dynamics, but collisions)
    c->restitution = 1;*/

    // Create obstacle circle physics body
    PhysicsBody circle = CreatePhysicsBodyPolygon(state, (Vector2){SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2}, GetRandomValue(20, 80), GetRandomValue(3, 8), 10);
    // CreatePhysicsBodyCircle(state,(Vector2){ SCREEN_WIDTH/2, SCREEN_HEIGHT/2 }, 45, 10);
    circle->enabled = true; // Disable body state to convert it to static (no dynamics, but collisions)
    // circle->restitution = 1;
    circle->velocity = (Vector2){0, -1};

    SetPhysicsGravity(state, 0, 9);
    // SetPhysicsTimeStep(0.1);

    SDL_Surface *screen_surface = NULL;

    SDL_Window *window = NULL;
    SDL_Renderer *render = NULL;

    window = SDL_CreateWindow("Hello, SDL 2!", SDL_WINDOWPOS_UNDEFINED,
                              SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT,
                              SDL_WINDOW_SHOWN);

    if (window == NULL)
    {
        return 1;
    }

    // screen_surface = SDL_GetWindowSurface(window);
    render = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    // SDL_FillRect(screen_surface, NULL, SDL_MapRGB( screen_surface->format, 0, 255, 0));

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

            if (event.type == SDL_MOUSEBUTTONDOWN)
            {
                // do whatever you want to do after a mouse button was pressed,
                //  e.g.:
                mousePress(state, event.button);
            }
        }

        for (int i = 0; i < state->manifoldsCount; i++)
        {
            PhysicsManifoldData m = state->manifolds[i];
            // printf("%d",m->contactsCount);
            if (m.bodyA == circle && m.contactsCount > 0 && m.bodyB != floor)
            {
                // DestroyPhysicsBody(state,m.bodyB);
                //printf("%d", m.contactsCount);
                // PhysicsAddForce(m.bodyB, (Vector2){0,-1000});
                m.bodyB->velocity = m.normal;
                // break;
            }
            else if (m.bodyB == circle && m.contactsCount > 0 && m.bodyA != floor)
            {
                // DestroyPhysicsBody(state,m.bodyA);
                // printf("%d",m.contactsCount);
                //  printf("shatter");
                // break;
                // Physics(m.bodyA, (Vector2){0,-1000});
                m.bodyA->velocity = m.normal;
            }
        }
        PhysicsStep(state);
        int bodiesCount = GetPhysicsBodiesCount(state);
        for (int i = bodiesCount - 1; i >= 0; i--)
        {
            PhysicsBody body = GetPhysicsBody(state, i);

            if ((body != NULL) && (body->position.y > SCREEN_HEIGHT * 2))
                DestroyPhysicsBody(state, body);
        }

        SDL_SetRenderDrawColor(render, 0, 0, 0, 0);
        SDL_RenderClear(render);

        bodiesCount = GetPhysicsBodiesCount(state);

        for (int i = 0; i < bodiesCount; i++)
        {
            SDL_SetRenderDrawColor(render, 255, 0, 0, 0);

            PhysicsBody body = GetPhysicsBody(state, i);
            if (body == circle)
            {
                SDL_SetRenderDrawColor(render, 0, 255, 0, 0);
            }
            if (body != NULL)
            {
                int vertexCount = GetPhysicsShapeVerticesCount(state, i);
                for (int j = 0; j < vertexCount; j++)
                {
                    // Get physics bodies shape vertices to draw lines
                    // Note: GetPhysicsShapeVertex() already calculates rotation transformations
                    Vector2 vertexA = GetPhysicsShapeVertex(body, j);
                    int jj = (((j + 1) < vertexCount) ? (j + 1) : 0); // Get next vertex or first to close the shape
                    Vector2 vertexB = GetPhysicsShapeVertex(body, jj);
                    SDL_RenderDrawLineF(render, vertexA.x, vertexA.y, vertexB.x, vertexB.y); // Draw a line between two vertex positions
                }
            }
        }
        SDL_RenderPresent(render);
        SDL_Delay(1000 / SDL_GetTicks());
        /*
         update_all(bodies,N);

          render_all(render, bodies,N);*/
    }

    // SDL_Delay(2000);
    ClosePhysics(state);
    SDL_DestroyWindow(window);
    SDL_DestroyRenderer(render);
    SDL_Quit();

    return 0;
};