enum constants{
    MAX_VERTICES = 20,
    MAX_BODIES = 20
};

typedef struct vec{
    float x, y;
} vec;

inline void vec_add(struct vec* a, struct vec* b){
    a->x += b->x;
    b->y += b->y;
}

struct vec add(struct vec a, struct vec b){
   // printf("\n qq%f %f \n", a.x,b.x);
    return (struct vec){a.x+b.x, a.y+b.y};
}

vec sub(vec a, vec b){
    return (vec){a.x-b.x, a.y-b.y};
}

float dot(vec a, vec b){
    return a.x*b.x+a.y*b.y;
};

vec perp(vec v){
    return (vec){v.y, -v.x};
}

struct rect
{
    struct vec position;
    struct vec size;
};


typedef struct polygon
{
    struct vec vertices [MAX_VERTICES];
    int n;
    struct vec transformed_vertices [MAX_VERTICES];
} polygon;

struct body{
    struct vec position;
    struct polygon shape;
    struct rect AABB;
    float mass;
    struct vec velocity;
    float rot_velocity;
    float angle;
    int intersect;
};

int which_side(polygon  C, vec P, vec D){
    int positive=0, negative=0;
    for(int i=0; i<C.n; i++){
        float t = dot(D, sub(C.transformed_vertices[i],P));
        if(t > 0){
            positive++;
        } else if(t < 0){
            negative++;
        }
        if(positive && negative) return 0;
        return (positive>0 ? +1:-1);
    }
    
};

int intersect(polygon C0,polygon C1) {

    for(int i0=0, i1=C0.n-1; i0<C0.n; i1=i0, i0++){
        vec P =C0.transformed_vertices[i0];
        vec D = perp(sub(C0.transformed_vertices[i0], C0.transformed_vertices[i1]));//outwardpointing
        if(which_side(C1,P,D)>0){
            return 0;
        }
    }

    for(int i0=0,i1=C1.n-1; i0<C1.n; i1=i0, i0++){
        vec P = C1.transformed_vertices[i0];
        vec D = perp(sub(C1.transformed_vertices[i0],C1.transformed_vertices[i1]));//outwardpointing
        if(which_side(C0,P,D)>0){
            return 0;
        }
    }
    return 1;
}
struct result{
    //TODO: aaa
};

struct result process_collision(struct body* b1, struct body* b2){
    //TODO:
}


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
    
    body->position = add(body->position, body->velocity);
    body->angle += body->rot_velocity;

    float a = body->angle;

    for(int i=0;i<body->shape.n;i++){
        struct vec point = body->shape.vertices[i];

        float X,Y;
    
        X=point.x*cos(a)-point.y*sin(a);
        Y=point.y*cos(a)+point.x*sin(a);
        body->shape.transformed_vertices[i] = add(body->position,(struct vec){X,Y});
    }
};

void delete_body(struct body* body){
    free(body);
}

void update_all(struct body ** bodies, int N){
    for(int i=0; i< N; i++){
        update_body(bodies[i]);
        for(int j=i+1; j< N; j++){
            int flag = intersect(bodies[i]->shape, bodies[j]->shape);
            bodies[i]->intersect = flag;
            bodies[j]->intersect = flag;
        }
    }
}