
#define FILTER_FIR_I16_LENGTH 32          // maximum history our filter will keep
#define FILTER_FIR_I16_COEF_SHIFT 12      // filter coefs are 3.12 fixed point
// generate coefficient from floating point value. Preprocessor use only!
#define FILTER_FIR_I16_COEF(coef) ((int16_t)(((coef) + ((coef) < 0 ? -1 : 1) * (1 << (FILTER_FIR_I16_COEF_SHIFT - 1))) * (1 << FILTER_FIR_I16_COEF_SHIFT)))

typedef struct {
    uint16_t head;
    int16_t state[FILTER_FIR_I16_LENGTH];
} filterStateFIRi16_t;

typedef struct {
    int taps;
    int16_t coef[];
} filterConfigFIRi16_t;

int16_t filterApplyFIRi16(int16_t input, filterStateFIRi16_t *state, const filterConfigFIRi16_t *config);
void filterResetFIRi16(int16_t value, filterStateFIRi16_t *state, const filterConfigFIRi16_t *config);
