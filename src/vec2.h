
typedef struct vec2_Struct
{
    float x;
    float y;
} vec2;

static inline vec2 vec2_add(vec2 a, vec2 b)
{
    a.x += b.x;
    a.y += b.y;
    return a;
}

static inline vec2 vec2_subtract(vec2 a, vec2 b)
{
    a.x -= b.x;
    a.y -= b.y;
    return a;
}

static inline vec2 vec2_scalar_multiply(vec2 a, float b)
{
    a.x *= b;
    a.y *= b;
    return a;
}

static inline float vec2_dot_product(vec2 a, vec2 b)
{
    return a.x * b.x + a.y * b.y; 
}

static inline float vec2_length(vec2 a)
{
    return sqrt(a.x * a.x + a.y * a.y);
}

static inline vec2 vec2_normalize(vec2 a)
{
    float scale = sqrt( a.x * a.x + a.y * a.y);
    a.x /= scale;
    a.y /= scale;

    return a;
}

static inline vec2 vec2_make(float a, float b)
{
    vec2 temp;
    temp.x = a;
    temp.y = b;
    return temp;
}

static inline float vec2_angle_degrees(vec2 a)
{
    float angle_radians = atan2(a.y, a.x);
    return (angle_radians / M_PI) * 180.0;
}

static inline vec2 vec2_rotate(vec2 a, float b)
{
    float theta = b * 180.0 / M_PI;

    float cs = cos(theta);
    float sn = sin(theta);

    vec2 temp;
    temp.x = a.x * cs - a.y * sn; 
    temp.y = a.x * sn + a.y * cs;

    return temp;
}

static inline vec2 vec2_lerp(vec2 a, vec2 b, float t)
{
    return vec2_add(vec2_scalar_multiply(a, 1-t), vec2_scalar_multiply(b, t));
}
