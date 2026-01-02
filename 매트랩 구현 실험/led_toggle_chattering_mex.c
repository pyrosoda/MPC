/*
 * led_toggle_chattering_mex.c
 * 실습 1: LED 토글(채터링 방지) - MATLAB MEX 용
 *
 * 사용법 (MATLAB):
 *   mex led_toggle_chattering_mex.c
 *   led = led_toggle_chattering_mex(btn);        % btn: 0 or 1
 *   [led, dbg] = led_toggle_chattering_mex(btn); % dbg: 디바운스 상태 확인용(선택)
 *
 * 동작 개요:
 * - 버튼 입력(btn)이 연속으로 N번 동일하게 유지될 때만 "안정된 상태"로 인정
 * - 안정된 상태가 0->1 (상승엣지)로 바뀌는 순간에만 LED 토글
 */

#include "mex.h"
#include <stdint.h>

/* ===== 튜닝 파라미터 =====
 * MATLAB에서 이 함수를 "몇 초/몇 ms 간격으로" 호출하는지에 따라 조절합니다.
 * 예) 1ms마다 호출이면 20이면 20ms 디바운스
 * 예) 10ms마다 호출이면 5면 50ms 디바운스
 */
#define DEBOUNCE_COUNT  10

/* ===== 내부 상태(호출 간 유지) ===== */
static uint8_t led_state = 0;          // 최종 LED 상태(0/1)
static uint8_t stable_btn = 0;         // 디바운싱 후 안정된 버튼 상태(0/1)
static uint8_t last_stable_btn = 0;    // 직전 안정 버튼 상태
static uint8_t last_raw_btn = 0;       // 직전 raw 입력
static uint16_t same_count = 0;        // raw 입력이 연속으로 동일한 횟수

/* MATLAB에서 mex를 다시 로드할 때 상태 리셋하고 싶으면, clear mex로 초기화 가능 */

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

    /* ===== 디바운싱: raw 입력이 연속으로 같은 값이면 same_count 증가 ===== */
    if (raw_btn == last_raw_btn) {
        if (same_count < 65535) same_count++;
    } else {
        same_count = 1;
        last_raw_btn = raw_btn;
    }

    /* ===== 충분히 안정적이면 stable_btn 갱신 ===== */
    if (same_count >= DEBOUNCE_COUNT) {
        stable_btn = raw_btn;
    }

    /* ===== 상승엣지에서만 토글 ===== */
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
