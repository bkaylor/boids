#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include "SDL.h"
#include "SDL_ttf.h"
#include "SDL_image.h"

#include "vec2.h"
#include "draw.h"

typedef struct {
    int x;
    int y;
} Window;

typedef struct {
    vec2 position;
    vec2 velocity;

    vec2 delta_to_nearest_boid;

    vec2 center_force;
    vec2 avoid_force;
    vec2 match_force;
    vec2 wall_force;
} Boid;

#define BOID_COUNT 50
#define MAX_BOIDS 1000
typedef struct {
    Window window;
    Boid boids[MAX_BOIDS];
    int boid_count;
    int boid_count_max;

    vec2 center_of_mass;

    bool show_debug_lines;

    bool reset;
    bool quit;
} State;

void render(SDL_Renderer *renderer, State state)
{
    SDL_RenderClear(renderer);

    // Set background color.
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderFillRect(renderer, NULL);

    if (state.show_debug_lines)
    {
        // Draw the nearest-boid lines.
        SDL_SetRenderDrawColor(renderer, 127, 127, 127, 255);
        for (int i = 0; i < state.boid_count; i += 1)
        {
            Boid boid = state.boids[i];
            SDL_RenderDrawLine(renderer, boid.position.x, boid.position.y, 
                               boid.position.x - boid.delta_to_nearest_boid.x, 
                               boid.position.y - boid.delta_to_nearest_boid.y);
        }

        // Draw the force lines.
        for (int i = 0; i < state.boid_count; i += 1)
        {
            Boid boid = state.boids[i];

            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
            SDL_RenderDrawLine(renderer, boid.position.x, boid.position.y, 
                               boid.position.x - boid.center_force.x, 
                               boid.position.y - boid.center_force.y);

            SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
            SDL_RenderDrawLine(renderer, boid.position.x, boid.position.y, 
                               boid.position.x - boid.avoid_force.x, 
                               boid.position.y - boid.avoid_force.y);

            SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
            SDL_RenderDrawLine(renderer, boid.position.x, boid.position.y, 
                               boid.position.x - boid.match_force.x, 
                               boid.position.y - boid.match_force.y);

#if 0
            SDL_SetRenderDrawColor(renderer, 0, 127, 127, 255);
            SDL_RenderDrawLine(renderer, boid.position.x, boid.position.y, 
                               boid.position.x - boid.wall_force.x, 
                               boid.position.y - boid.wall_force.y);
#endif
        }
    }


    // Draw the boids.
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    for (int i = 0; i < state.boid_count; i += 1)
    {
        Boid boid = state.boids[i];
        SDL_RenderDrawLine(renderer, boid.position.x, boid.position.y, boid.position.x + 10*boid.velocity.x, boid.position.y + 10*boid.velocity.y);
    }

    // Draw center of mass.
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    draw_circle(renderer, state.center_of_mass.x, state.center_of_mass.y, 10);

    SDL_RenderPresent(renderer);
}

void update(State *state) 
{
    if (state->reset)
    {
        state->boid_count = 0;
        for (int i = 0; i < state->boid_count_max; i += 1)
        {
            Boid *boid = &state->boids[i];
            boid->position = vec2_make((float)(rand() % state->window.x), (float)(rand() % state->window.y));
            boid->velocity = vec2_rotate(vec2_normalize(vec2_make(1.0f, 1.0f)), (float)(rand() % 6));
            state->boid_count += 1;
        }

        state->reset = false;
    }

    // Get center of all boids
    vec2 center_of_mass = vec2_make(0.0f, 0.0f);
    for (int i = 0; i < state->boid_count; i += 1)
    {
        Boid *boid = &state->boids[i];
        center_of_mass = vec2_add(center_of_mass, boid->position);
    }
    state->center_of_mass = vec2_scalar_multiply(center_of_mass, (float)1/state->boid_count);

    for (int i = 0; i < state->boid_count; i += 1)
    {
        Boid *boid = &state->boids[i];

        //
        // Fly toward center
        //
        {
            vec2 delta_to_center = vec2_subtract(state->center_of_mass, boid->position);
            float percent_to_nudge = 0.00003;
            boid->velocity = vec2_normalize(vec2_lerp(boid->velocity, delta_to_center, percent_to_nudge));

            boid->center_force = vec2_scalar_multiply(delta_to_center, percent_to_nudge);
        }

        //
        // Avoid others
        //
        {
            // Get closest boid.
            int closest_boid_id = -1;
            float min_distance = 100000.0f;
            vec2 min_delta = vec2_make(1.0f, 1.0f);
            for(int j = 0; j < state->boid_count; j += 1)
            {
                if (i == j) continue;

                Boid inner_boid = state->boids[j];
                vec2 delta = vec2_subtract(boid->position, inner_boid.position);

                float distance = delta.x * delta.x + delta.y * delta.y;
                if (distance < min_distance) 
                {
                    min_distance = distance;
                    min_delta = delta;
                }
            }

            float percent_to_nudge = 0.01;
            boid->velocity = vec2_normalize(vec2_lerp(boid->velocity, vec2_normalize(min_delta), percent_to_nudge));
            boid->delta_to_nearest_boid = min_delta;

            boid->avoid_force = vec2_scalar_multiply(min_delta, percent_to_nudge);
        }
        
        //
        // Match velocity
        //
        {
            // Get closest boid.
            int closest_boid_id = -1;
            float min_distance = 100000.0f;
            vec2 min_delta = vec2_make(1.0f, 1.0f);
            int min_id = -1;
            for(int j = 0; j < state->boid_count; j += 1)
            {
                if (i == j) continue;

                Boid inner_boid = state->boids[j];
                vec2 delta = vec2_subtract(boid->position, inner_boid.position);

                float distance = delta.x * delta.x + delta.y * delta.y;
                if (distance < min_distance) 
                {
                    min_distance = distance;
                    min_delta = delta;
                    min_id = j;
                }
            }

            float percent_to_nudge = 0.01;
            boid->velocity = vec2_normalize(vec2_lerp(boid->velocity, state->boids[min_id].velocity, percent_to_nudge));
            boid->delta_to_nearest_boid = min_delta;

            boid->match_force = vec2_scalar_multiply(state->boids[min_id].velocity, percent_to_nudge);
        }

        //
        // Avoid walls.
        //
#if 0
        {
            float top, left, right, bottom;
            top = boid->position.y;
            left = boid->position.x;
            right = state->window.x - boid->position.x;
            bottom = state->window.y - boid->position.y;

            enum wall_direction {
                T, L, R, B
            };

            enum wall_direction dir = top;
            float min = top;

            if (left < min)
            {
                dir = L;
                min = left;
            }

            if (right < min)
            {
                dir = R;
                min = right;
            }

            if (bottom < min)
            {
                dir = B;
                min = bottom;
            }

            if (min < 0) min = 0.1;

            vec2 normal = vec2_make(0.0f, 0.0f);
            if (dir == T)
            {
                normal = vec2_make(0.0f, 1.0f);
            }
            else if (dir == L)
            {
                normal = vec2_make(1.0f, 0.0f);
            }
            else if (dir == R)
            {
                normal = vec2_make(-1.0f, 0.0f);
            }
            else
            {
                normal = vec2_make(0.0f, -1.0f);
            }

            normal = vec2_scalar_multiply(normal, (float)1/min);
            normal.x = normal.x * normal.x;
            normal.y = normal.y * normal.y;

            float percent_to_nudge = 0.07f;
            boid->velocity = vec2_normalize(vec2_lerp(boid->velocity, normal, percent_to_nudge));

            boid->wall_force = vec2_scalar_multiply(normal, percent_to_nudge);
        }
#endif

        boid->center_force = vec2_normalize(boid->center_force);
        boid->avoid_force = vec2_normalize(boid->avoid_force);
        boid->match_force = vec2_normalize(boid->match_force);
        boid->wall_force = vec2_normalize(boid->wall_force);

        // Update
        boid->position = vec2_add(boid->position, boid->velocity);

        // Wrap around walls
        if (boid->position.x > state->window.x) 
        {
            boid->position.x = 0;
        }
        if (boid->position.x < 0) 
        {
            boid->position.x = state->window.x;
        }
        if (boid->position.y > state->window.y) 
        {
            boid->position.y = 0;
        }
        if (boid->position.y < 0) 
        {
            boid->position.y = state->window.y;
        }
    }

    return;
}

void get_input(State *state)
{
    // Handle events.
    SDL_Event event;

    while (SDL_PollEvent(&event))
    {
        switch (event.type)
        {
            case SDL_KEYDOWN:
                switch (event.key.keysym.sym)
                {
                    case SDLK_ESCAPE:
                        state->quit = true;
                        break;

                    case SDLK_SPACE:
                        break;

                    case SDLK_TAB:
                        state->show_debug_lines = !state->show_debug_lines;
                        break;
                    
                    case SDLK_r:
                        state->reset = true;
                        break;

                    case SDLK_UP:
                        break;

                    case SDLK_DOWN:
                        break;

                    default:
                        break;
                }
                break;

            case SDL_QUIT:
                state->quit = true;
                break;

            default:
                break;
        }
    }
}

int main(int argc, char *argv[])
{
	SDL_Init(SDL_INIT_EVERYTHING);
    IMG_Init(IMG_INIT_PNG);

    if (SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        printf("SDL_Init video error: %s\n", SDL_GetError());
        return 1;
    }

    if (SDL_Init(SDL_INIT_AUDIO) != 0)
    {
        printf("SDL_Init audio error: %s\n", SDL_GetError());
        return 1;
    }

    // SDL_ShowCursor(SDL_DISABLE);

	// Setup window
	SDL_Window *win = SDL_CreateWindow("Boids",
			SDL_WINDOWPOS_CENTERED,
			SDL_WINDOWPOS_CENTERED,
			1440, 980,
			SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

	// Setup renderer
	SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

	// Setup font
	TTF_Init();
	TTF_Font *font = TTF_OpenFont("liberation.ttf", 12);
	if (!font)
	{
		SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Error: Font", TTF_GetError(), win);
		return -666;
	}

    // Setup main loop
    srand(time(NULL));

    // Main loop
    const float FPS_INTERVAL = 1.0f;
    Uint64 fps_start, fps_current, fps_frames = 0;

    fps_start = SDL_GetTicks();

    State state;
    state.reset = true;
    state.quit = false;
    state.boid_count = 0;
    state.boid_count_max = BOID_COUNT;
    state.center_of_mass = vec2_make(0.0f, 0.0f);
    state.show_debug_lines = false;

    while (!state.quit)
    {
        SDL_PumpEvents();
        get_input(&state);

        if (!state.quit)
        {
            int x, y;
            SDL_GetWindowSize(win, &state.window.x, &state.window.y);

            update(&state);
            render(ren, state);

            fps_frames++;

            if (fps_start < SDL_GetTicks() - FPS_INTERVAL * 1000)
            {
                fps_start = SDL_GetTicks();
                fps_current = fps_frames;
                fps_frames = 0;

                // printf("%I64d fps\n", fps_current);
            }
        }
    }

	SDL_DestroyRenderer(ren);
	SDL_DestroyWindow(win);
	SDL_Quit();
    return 0;
}
