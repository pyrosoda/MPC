#include "mex.h"
#include <stdint.h>


#define DEBOUNCE_COUNT  10

/* ===== 내부 상태(호출 간 유지) ===== */
static uint8_t led_state = 0;          // 최종 LED 상태(0/1)
static uint8_t stable_btn = 0;         // 디바운싱 후 안정된 버튼 상태(0/1)
static uint8_t last_stable_btn = 0;    // 직전 안정 버튼 상태
static uint8_t last_raw_btn = 0;       // 직전 raw 입력
static uint16_t same_count = 0;        // raw 입력이 연속으로 동일한 횟수


static uint8_t clamp01(double x)
{
    return (x >= 0.5) ? 1u : 0u;
}

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    /* 입력 체크 */
    if (nrhs != 1) {
        mexErrMsgIdAndTxt("LED:Input",
                          "입력은 1개 필요합니다: btn (0 또는 1)");
    }
    if (!mxIsNumeric(prhs[0]) || mxIsComplex(prhs[0]) || mxGetNumberOfElements(prhs[0]) != 1) {
        mexErrMsgIdAndTxt("LED:InputType",
                          "btn은 실수 스칼라(0/1)여야 합니다.");
    }

    double btn_in = mxGetScalar(prhs[0]);
    uint8_t raw_btn = clamp01(btn_in);

    if (raw_btn == last_raw_btn) {
        if (same_count < 65535) same_count++;
    } else {
        same_count = 1;
        last_raw_btn = raw_btn;
    }
    if (same_count >= DEBOUNCE_COUNT) {
        stable_btn = raw_btn;
    }

    if (last_stable_btn == 0 && stable_btn == 1) {
        led_state ^= 1u;  // 토글
    }
    last_stable_btn = stable_btn;

    /* 출력 1: LED 상태 */
    plhs[0] = mxCreateDoubleScalar((double)led_state);

    /* 출력 2(선택): 디버그 정보 [raw_btn, stable_btn, same_count] */
    if (nlhs >= 2) {
        mxArray *dbg = mxCreateDoubleMatrix(1, 3, mxREAL);
        double *p = mxGetPr(dbg);
        p[0] = (double)raw_btn;
        p[1] = (double)stable_btn;
        p[2] = (double)same_count;
        plhs[1] = dbg;
    }
}

