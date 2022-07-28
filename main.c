#include <SDL2/SDL.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;

const int MAX_VERTICES = 20;
struct vec{
    float x, y;
};
struct mat{
    float x1,x2,y1,y2;
};

inline void vec_add(struct vec* a, struct vec* b){
    a->x += b->x;
    b->y += b->y;
}

inline struct vec add(struct vec a, struct vec b){
   // printf("\n qq%f %f \n", a.x,b.x);
    return {a.x+b.x, a.y+b.y};
}

struct rect
{
    struct vec position;
    struct vec size;
};


struct polygon
{
    struct vec vertices [MAX_VERTICES];
    int n;
    struct vec transformed_vertices [MAX_VERTICES];
};

struct body{
    struct vec position;
    struct polygon shape;
    struct rect AABB;
    float mass;
    struct vec velocity;
    float rot_velocity;
    float angle = 0;
};

struct body* new_body(struct vec pos, int n, struct vec vertices[]){
    struct body* body = (struct body*)malloc(sizeof(struct body));
    for(int i=0; i<n; i++){
        body->shape.vertices[i] = vertices[i];
    }
    body->shape.n = n;
    body->position = pos;

    return body;
};


void update_body(struct body* body){
    //body->position = add(body->position, body->velocity);
    body->angle += body->rot_velocity;

    struct vec origin = body->position;
    float angle = body->angle;

    //struct vec* p = body->shape.transformed_vertices;
    //struct vec* transformed = body->shape.transformed_vertices;
    for(int i=0;i<body->shape.n;i++){
        //body->shape.transformed_vertices[i] = add(body->position, body->shape.vertices[i]);
        struct vec point = body->shape.vertices[i];
        float X = origin.x + ((point.x - origin.x) * cos(angle) -
		(point.y - origin.y) * sin(angle));
	    float Y = origin.y +  ((point.x - origin.x) * sin(angle) +
		(point.y - origin.y) * cos(angle));
	
        body->shape.transformed_vertices[i] = {X,Y};
    }
};

void delete_body(struct body* body){
    free(body);
}


/*
void render_body(SDL_Renderer* render, struct body* body){
    SDL_SetRenderDrawColor(render,255,0,0,0);
}*/
void render_body(SDL_Renderer* render, struct body* body){
    SDL_SetRenderDrawColor(render,255,0,0,0);
    int n = body->shape.n;
    struct vec* transformed = body->shape.transformed_vertices;

    //struct vec* p = body.shape.transformed_vertices;
    float x,y;
    for(int i=0; i<n;i++){ 
        struct vec* p = &transformed[i];
        
        x = p->x;
        y = p->y;
        printf("%f %f \n", x, y);

        //SDL_FillRect(screen_surface, &rect, SDL_MapRGB( screen_surface->format, 255, 0, 0));
    }
    transformed = body->shape.transformed_vertices;
    SDL_RenderDrawLinesF( render, (const SDL_FPoint*) transformed, n );
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

void update_all(struct body ** bodies, int N){
    for(int j=0;j<N;j++){
        update_body(bodies[j]);
    }
}


int main (int argc, char ** args) {
    
    if( SDL_Init( SDL_INIT_EVERYTHING ) != 0 )
    {
        return 1;
    }

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

    

    //SDL_AddTimer(100,my_callbackfunc,render);
    struct vec vertices[]= {
        {100,100},
        {130,110},
        {150,150}
    };
    const int N=2;
    struct body* bodies[N] = {
        new_body({1,1},3,vertices),
        new_body({100,100},3,vertices)
    };
    bodies[0]->velocity={1,1};
    bodies[0]->rot_velocity=0.01f;
    bodies[1]->velocity = {-1,-1};
    struct body* body = new_body({1,1}, 3, vertices);

    struct vec transformed[]= {
        {100,100},
        {130,110},
        {150,150}
    };

    struct body** p = bodies;

    while (1)
    {
        update_all(bodies,N);

        render_all(render, bodies,N);
        /*SDL_SetRenderDrawColor(render,0,0,0,0);
        SDL_RenderClear(render);
        SDL_SetRenderDrawColor(render,255,0,0,0);

        update_body(body);
        render_body(render,body);

        SDL_RenderPresent(render); */

        SDL_Delay(100);

    }
    
    SDL_Delay(2000);
    
    SDL_DestroyWindow(window);
    SDL_DestroyRenderer(render);
    SDL_Quit();

    return 0;
};