#include QMK_KEYBOARD_H
#include "version.h"
#include "features/achordion.h"

#define MOON_LED_LEVEL LED_LEVEL
#define ML_SAFE_RANGE SAFE_RANGE

enum custom_keycodes {
  RGB_SLD = ML_SAFE_RANGE,
  ST_MACRO_0,
  ST_MACRO_1,
  ST_MACRO_2,
};



enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
  DANCE_2,
  DANCE_3,
  DANCE_4,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_voyager(
    KC_ESCAPE,      KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           KC_BSPC,
    KC_TAB,         KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,                                           KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           KC_MINUS,
    KC_LEFT_GUI,    KC_A,           KC_S,           KC_D,           LT(2,KC_F),     LT(1,KC_G),                                     KC_H,           KC_J,           KC_K,           KC_L,           KC_SCLN,        KC_QUOTE,
    MO(3),          MT(MOD_LALT, KC_Z),MT(MOD_LALT, KC_X),KC_C,           KC_V,           MT(MOD_LCTL, KC_B),                                MT(MOD_RSFT, KC_N),KC_M,           KC_COMMA,       KC_DOT,         MT(MOD_RALT, KC_SLASH),KC_RIGHT_CTRL,
                                                    KC_RIGHT_SHIFT, LT(1,KC_ENTER),                                 KC_SPACE,       MO(2)
  ),
  [1] = LAYOUT_voyager(
    KC_AUDIO_VOL_UP,KC_AUDIO_MUTE,  KC_MEDIA_PLAY_PAUSE,KC_MEDIA_PREV_TRACK,KC_MEDIA_NEXT_TRACK,QK_LLCK,                                        KC_PAGE_UP,     KC_HOME,        KC_NO,          KC_INSERT,      KC_DELETE,      KC_TRANSPARENT,
    KC_AUDIO_VOL_DOWN,KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_PGDN,        KC_END,         KC_UP,          KC_NO,          TD(DANCE_0),    KC_PLUS,
    KC_TRANSPARENT, KC_NO,          KC_NO,          KC_TRANSPARENT, LALT(KC_LEFT_CTRL),KC_TRANSPARENT,                                 KC_NO,          KC_LEFT,        KC_DOWN,        KC_RIGHT,       TD(DANCE_1),    KC_QUOTE,
    KC_NO,          KC_NO,          KC_NO,          KC_LEFT_SHIFT,  KC_LEFT_CTRL,   KC_LEFT_ALT,                                    KC_PSCR,        KC_NO,          KC_NO,          KC_NO,          KC_BSLS,        LSFT(KC_END),
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_RIGHT_SHIFT, KC_SPACE
  ),
  [2] = LAYOUT_voyager(
    KC_ESCAPE,      KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,                                          KC_F6,          KC_F7,          TD(DANCE_3),    KC_F9,          KC_F10,         KC_F11,
    KC_TRANSPARENT, KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_LPRN,        KC_DQUO,        KC_RPRN,        KC_NO,          KC_EQUAL,       KC_F12,
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          TD(DANCE_2),    LCTL(KC_G),                                     KC_LCBR,        KC_NO,          KC_RCBR,        KC_NO,          KC_NO,          KC_TRANSPARENT,
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          LCTL(LSFT(KC_B)),                                KC_LBRC,        KC_PIPE,        KC_RBRC,        KC_NO,          KC_BSLS,        KC_NO,
                                                    KC_TRANSPARENT, KC_LEFT_CTRL,                                   KC_LEFT_ALT,    KC_TRANSPARENT
  ),
  [3] = LAYOUT_voyager(
    LGUI(LSFT(KC_F23)),RGB_TOG,        TOGGLE_LAYER_COLOR,RGB_MODE_FORWARD,RGB_SLD,        RGB_VAD,                                        ST_MACRO_0,     ST_MACRO_1,     ST_MACRO_2,     KC_NO,          LSFT(KC_INSERT),LCTL(LSFT(KC_DELETE)),
    KC_TRANSPARENT, LCTL(KC_Q),     LCTL(KC_W),     LCTL(KC_E),     LCTL(KC_R),     RGB_VAI,                                        LALT(LCTL(KC_UP)),KC_NO,          KC_NO,          KC_NO,          TD(DANCE_4),    KC_NO,
    QK_BOOT,        LCTL(KC_A),     LCTL(KC_S),     LCTL(KC_D),     KC_TRANSPARENT, KC_TRANSPARENT,                                 LALT(LCTL(KC_DOWN)),KC_NO,          KC_NO,          LALT(KC_BSLS),  LALT(KC_RBRC),  LCTL(KC_RIGHT),
    KC_ESCAPE,      LCTL(KC_Z),     LCTL(KC_X),     LCTL(KC_C),     LCTL(KC_V),     LCTL(KC_B),                                     LCTL(KC_N),     LCTL(LSFT(KC_TAB)),LCTL(KC_TAB),   KC_NO,          KC_NO,          KC_NO,
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 LCTL(LSFT(KC_ENTER)),KC_TRANSPARENT
  ),
  [4] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_NO,          KC_HOME,        KC_UP,          KC_END,         KC_PAGE_UP,     KC_TRANSPARENT,
    KC_NO,          KC_LEFT_ALT,    KC_LEFT_CTRL,   KC_LEFT_SHIFT,  KC_NO,          KC_TRANSPARENT,                                 KC_NO,          KC_LEFT,        KC_DOWN,        KC_RIGHT,       KC_PGDN,        KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
};

const uint16_t PROGMEM combo0[] = { KC_8, KC_9, COMBO_END};
const uint16_t PROGMEM combo1[] = { KC_9, KC_0, COMBO_END};
const uint16_t PROGMEM combo2[] = { KC_I, KC_9, COMBO_END};
const uint16_t PROGMEM combo3[] = { KC_9, KC_P, COMBO_END};
const uint16_t PROGMEM combo4[] = { KC_COMMA, KC_DOT, COMBO_END};
const uint16_t PROGMEM combo5[] = { KC_DOT, MT(MOD_RALT, KC_SLASH), COMBO_END};
const uint16_t PROGMEM combo6[] = { KC_P, KC_MINUS, COMBO_END};

combo_t key_combos[COMBO_COUNT] = {
    COMBO(combo0, KC_LCBR),
    COMBO(combo1, KC_RCBR),
    COMBO(combo2, KC_LPRN),
    COMBO(combo3, KC_RPRN),
    COMBO(combo4, KC_GRAVE),
    COMBO(combo5, KC_TILD),
    COMBO(combo6, KC_EQUAL),
};

uint16_t get_tapping_term(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case KC_S:
            return TAPPING_TERM -100;
        case KC_RIGHT_SHIFT:
            return TAPPING_TERM -80;
        case LT(1,KC_ENTER):
            return TAPPING_TERM + 120;
        case KC_MINUS:
            return TAPPING_TERM -5;
        case TD(DANCE_1):
            return TAPPING_TERM -30;
        default:
            return TAPPING_TERM;
    }
}

extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [0] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {171,142,200}, {171,142,200}, {171,142,200}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {239,218,204}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {239,218,204} },

    [1] = { {210,218,204}, {210,218,204}, {208,255,255}, {208,255,255}, {208,255,255}, {0,0,0}, {208,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {54,213,255}, {54,213,255}, {54,213,255}, {0,0,0}, {0,0,0}, {206,255,255}, {208,255,255}, {0,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {206,255,255}, {208,255,255}, {208,255,255}, {0,0,0}, {125,255,255}, {0,0,0}, {0,255,255}, {208,255,255}, {208,255,255}, {208,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {54,218,204}, {152,255,255}, {0,0,0}, {0,0,0} },

    [2] = { {0,0,0}, {84,218,204}, {84,218,204}, {84,218,204}, {84,218,204}, {84,218,204}, {0,0,0}, {0,0,0}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,0,0}, {0,255,255}, {0,255,255}, {0,255,255}, {0,0,0}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {84,218,204}, {84,218,204}, {84,218,204}, {84,218,204}, {84,218,204}, {84,218,204}, {0,0,0}, {0,0,0}, {0,0,0}, {0,255,255}, {0,0,0}, {84,218,204}, {0,0,0}, {0,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,255}, {0,0,0}, {0,255,255}, {0,0,255}, {0,0,0}, {0,0,0}, {0,0,0} },

    [3] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,255,255}, {54,213,255}, {91,218,204}, {0,0,0}, {54,213,255}, {54,213,255}, {171,142,200}, {0,0,0}, {0,0,0}, {0,0,0}, {54,213,255}, {0,0,0}, {171,142,200}, {0,0,0}, {0,0,0}, {54,213,255}, {54,213,255}, {54,213,255}, {0,0,0}, {91,218,204}, {91,218,204}, {0,0,0}, {0,0,0}, {0,0,0}, {171,218,204}, {0,0,0} },

    [4] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,218,204}, {0,0,0}, {0,0,0}, {0,0,0}, {0,218,204}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,218,204}, {54,213,255}, {91,218,204}, {54,213,255}, {54,213,255}, {0,0,0}, {0,218,204}, {54,218,204}, {54,218,204}, {54,218,204}, {54,213,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

};

void matrix_scan_user(void) {
  achordion_task();
}

void set_layer_color(int layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb( hsv );
        float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
        rgb_matrix_set_color( i, f * rgb.r, f * rgb.g, f * rgb.b );
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (rawhid_state.rgb_control) {
      return false;
  }
  if (keyboard_config.disable_layer_led) { return false; }
  switch (biton32(layer_state)) {
    case 0:
      set_layer_color(0);
      break;
    case 1:
      set_layer_color(1);
      break;
    case 2:
      set_layer_color(2);
      break;
    case 3:
      set_layer_color(3);
      break;
    case 4:
      set_layer_color(4);
      break;
   default:
    if (rgb_matrix_get_flags() == LED_FLAG_NONE)
      rgb_matrix_set_color_all(0, 0, 0);
    break;
  }
  return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  if (!process_achordion(keycode, record)) { return false; }

  switch (keycode) {
    case ST_MACRO_0:
    if (record->event.pressed) {
      SEND_STRING(SS_LCTL(SS_TAP(X_A)) SS_DELAY(100) SS_TAP(X_K));
    }
    break;
    case ST_MACRO_1:
    if (record->event.pressed) {
      SEND_STRING(SS_LCTL(SS_TAP(X_A)) SS_DELAY(100) SS_TAP(X_C));
    }
    break;
    case ST_MACRO_2:
    if (record->event.pressed) {
      SEND_STRING(SS_LCTL(SS_TAP(X_A)) SS_DELAY(100) SS_TAP(X_SPACE));
    }
    break;

    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
  }
  return true;
}


typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[5];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LCTL(KC_P));
        tap_code16(LCTL(KC_P));
        tap_code16(LCTL(KC_P));
    }
    if(state->count > 3) {
        tap_code16(LCTL(KC_P));
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(LCTL(KC_P)); break;
        case DOUBLE_TAP: register_code16(LCTL(LSFT(KC_P))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LCTL(KC_P)); register_code16(LCTL(KC_P));
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(LCTL(KC_P)); break;
        case DOUBLE_TAP: unregister_code16(LCTL(LSFT(KC_P))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LCTL(KC_P)); break;
    }
    dance_state[0].step = 0;
}
void on_dance_1(tap_dance_state_t *state, void *user_data);
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void on_dance_1(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_ESCAPE);
        tap_code16(KC_ESCAPE);
        tap_code16(KC_ESCAPE);
    }
    if(state->count > 3) {
        tap_code16(KC_ESCAPE);
    }
}

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(KC_ESCAPE); break;
        case DOUBLE_TAP: register_code16(KC_EXLM); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_ESCAPE); register_code16(KC_ESCAPE);
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(KC_ESCAPE); break;
        case DOUBLE_TAP: unregister_code16(KC_EXLM); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_ESCAPE); break;
    }
    dance_state[1].step = 0;
}
void on_dance_2(tap_dance_state_t *state, void *user_data);
void dance_2_finished(tap_dance_state_t *state, void *user_data);
void dance_2_reset(tap_dance_state_t *state, void *user_data);

void on_dance_2(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LCTL(KC_F));
        tap_code16(LCTL(KC_F));
        tap_code16(LCTL(KC_F));
    }
    if(state->count > 3) {
        tap_code16(LCTL(KC_F));
    }
}

void dance_2_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[2].step = dance_step(state);
    switch (dance_state[2].step) {
        case SINGLE_TAP: register_code16(LCTL(KC_F)); break;
        case SINGLE_HOLD: register_code16(LCTL(LSFT(KC_F))); break;
        case DOUBLE_TAP: register_code16(LCTL(KC_F)); register_code16(LCTL(KC_F)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LCTL(KC_F)); register_code16(LCTL(KC_F));
    }
}

void dance_2_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[2].step) {
        case SINGLE_TAP: unregister_code16(LCTL(KC_F)); break;
        case SINGLE_HOLD: unregister_code16(LCTL(LSFT(KC_F))); break;
        case DOUBLE_TAP: unregister_code16(LCTL(KC_F)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LCTL(KC_F)); break;
    }
    dance_state[2].step = 0;
}
void on_dance_3(tap_dance_state_t *state, void *user_data);
void dance_3_finished(tap_dance_state_t *state, void *user_data);
void dance_3_reset(tap_dance_state_t *state, void *user_data);

void on_dance_3(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_F8);
        tap_code16(KC_F8);
        tap_code16(KC_F8);
    }
    if(state->count > 3) {
        tap_code16(KC_F8);
    }
}

void dance_3_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[3].step = dance_step(state);
    switch (dance_state[3].step) {
        case SINGLE_TAP: register_code16(KC_F8); break;
        case SINGLE_HOLD: register_code16(LALT(KC_F8)); break;
        case DOUBLE_TAP: register_code16(KC_F8); register_code16(KC_F8); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_F8); register_code16(KC_F8);
    }
}

void dance_3_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[3].step) {
        case SINGLE_TAP: unregister_code16(KC_F8); break;
        case SINGLE_HOLD: unregister_code16(LALT(KC_F8)); break;
        case DOUBLE_TAP: unregister_code16(KC_F8); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_F8); break;
    }
    dance_state[3].step = 0;
}
void on_dance_4(tap_dance_state_t *state, void *user_data);
void dance_4_finished(tap_dance_state_t *state, void *user_data);
void dance_4_reset(tap_dance_state_t *state, void *user_data);

void on_dance_4(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_PAUSE);
        tap_code16(KC_PAUSE);
        tap_code16(KC_PAUSE);
    }
    if(state->count > 3) {
        tap_code16(KC_PAUSE);
    }
}

void dance_4_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[4].step = dance_step(state);
    switch (dance_state[4].step) {
        case SINGLE_TAP: register_code16(KC_PAUSE); break;
        case SINGLE_HOLD: register_code16(LCTL(KC_PSCR)); break;
        case DOUBLE_TAP: register_code16(KC_PAUSE); register_code16(KC_PAUSE); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_PAUSE); register_code16(KC_PAUSE);
    }
}

void dance_4_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[4].step) {
        case SINGLE_TAP: unregister_code16(KC_PAUSE); break;
        case SINGLE_HOLD: unregister_code16(LCTL(KC_PSCR)); break;
        case DOUBLE_TAP: unregister_code16(KC_PAUSE); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_PAUSE); break;
    }
    dance_state[4].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
        [DANCE_2] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_2, dance_2_finished, dance_2_reset),
        [DANCE_3] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_3, dance_3_finished, dance_3_reset),
        [DANCE_4] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_4, dance_4_finished, dance_4_reset),
};
