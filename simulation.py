import numpy as np 
import json
from typing import Optional, List, Tuple
import math

# ML stuff
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix

np.random.seed(42)

# constants
FREEFALL_WINDOW_MS   = 700
CONFIRM_WINDOW_MS    = 1500
POST_IMPACT_SAMPLES  = 20
STILLNESS_VARIANCE   = 2.5   # m/s²
SAMPLE_RATE_HZ       = 50    # 50 Hz
MS_PER_SAMPLE        = 1000 / SAMPLE_RATE_HZ   # 20 ms per sample

DEFAULT_IMPACT_THRESHOLD = 20.0
DEFAULT_JERK_THRESHOLD   = 3.0
DEFAULT_TILT_THRESHOLD   = 50.0
GRAVITY = 9.81


# FSM
class FSMResult:
    def __init__(self):
        self.detected          = False
        self.detection_sample  = None
        self.freefall_entered  = False
        self.impact_entered    = False
        self.still_check       = None   
        self.tilt_check        = None   


def run_fsm(acc_trace: np.ndarray,
            tilt_trace: np.ndarray,
            impact_threshold: float = DEFAULT_IMPACT_THRESHOLD,
            jerk_threshold: float   = DEFAULT_JERK_THRESHOLD,
            tilt_threshold: float   = DEFAULT_TILT_THRESHOLD) -> FSMResult:

    res = FSMResult()
    state            = "IDLE"
    state_entered_at = 0
    post_buf: List[float] = []
    last_alert_sample = -99999

    for i in range(len(acc_trace)):
        acc  = float(acc_trace[i])
        tilt = float(tilt_trace[i])
        elapsed_ms = (i - state_entered_at) * MS_PER_SAMPLE

        if state == "IDLE":
            if acc < jerk_threshold:
                state = "FREE_FALL"
                state_entered_at = i
                res.freefall_entered = True

        elif state == "FREE_FALL":
            if elapsed_ms > FREEFALL_WINDOW_MS:
                state = "IDLE"
            elif acc > impact_threshold:
                state = "IMPACT_SEEN"
                state_entered_at = i
                post_buf = []
                res.impact_entered = True

        elif state == "IMPACT_SEEN":
            if len(post_buf) < POST_IMPACT_SAMPLES:
                post_buf.append(acc)

            if elapsed_ms > CONFIRM_WINDOW_MS:
                arr      = np.array(post_buf)
                variance = float(np.var(arr))
                is_still  = variance < STILLNESS_VARIANCE
                is_tilted = tilt > tilt_threshold

                res.still_check = variance
                res.tilt_check  = tilt

                if is_still and is_tilted:
                    cooldown_ok = (i - last_alert_sample) * MS_PER_SAMPLE > 30000
                    if cooldown_ok:
                        last_alert_sample = i
                        res.detected         = True
                        res.detection_sample = i
                        state = "CONFIRMING"
                        state_entered_at = i
                        break
                else:
                    state = "IDLE"

        elif state == "CONFIRMING":
            if elapsed_ms > 30000:
                state = "IDLE"

    return res



def _noise(n, sigma): return np.random.normal(0, sigma, n)

# crash scenarios
def gen_crash(fall_type="lateral_slow", noise_sigma=None):
    if noise_sigma is None: noise_sigma = np.random.uniform(0.2, 0.7)
    N = 350
    acc  = np.full(N, GRAVITY)
    tilt = np.full(N, 5.0)

    ff_start, ff_len = 50, 17         
    ff_end = ff_start + ff_len    
    spike_s = ff_end               
    settle = spike_s + 1          

    if fall_type == "lateral_fast":
        ff_floor, impact_mag, final_tilt = np.random.uniform(0.2, 1.8), np.random.uniform(38, 75), np.random.uniform(65, 89)
    elif fall_type == "soft_surface":
        ff_floor, impact_mag, final_tilt = np.random.uniform(0.5, 2.2), np.random.uniform(22, 34), np.random.uniform(55, 85)
    elif fall_type == "forward":
        ff_floor, impact_mag, final_tilt = np.random.uniform(0.4, 2.6), np.random.uniform(27, 62), np.random.uniform(58, 88)
    else:  # lateral_slow
        ff_floor, impact_mag, final_tilt = np.random.uniform(0.3, 2.9), np.random.uniform(25.5, 48), np.random.uniform(60, 89)

    acc[:ff_start]  = GRAVITY + _noise(ff_start, noise_sigma * 0.3)
    tilt[:ff_start] = np.random.uniform(3, 8, ff_start)

    acc[ff_start:ff_end]  = np.linspace(GRAVITY, ff_floor, ff_len) + _noise(ff_len, noise_sigma * 0.2)
    tilt[ff_start:ff_end] = np.linspace(5, 30, ff_len) + _noise(ff_len, 1.0)

    acc[spike_s] = impact_mag + _noise(1, noise_sigma * 0.1)[0]

    post_len = N - settle
    acc[settle:]  = GRAVITY + _noise(post_len, noise_sigma * 0.25)
    tilt[settle:] = np.random.normal(final_tilt, 1.5, post_len)

    acc  += _noise(N, noise_sigma * 0.1)
    tilt += _noise(N, noise_sigma * 0.1)
    acc   = np.clip(acc, 0.0, None)
    return acc, tilt


# non-crash scenaerios
def gen_pothole(intensity="medium", noise_sigma=None):
    if noise_sigma is None: noise_sigma = np.random.uniform(0.2, 0.6)
    N = 200
    acc  = GRAVITY + _noise(N, noise_sigma)
    tilt = 4.0 + _noise(N, 0.5)

    spike_at = np.random.randint(20, 60)
    mag = {"mild": np.random.uniform(10, 22), "severe": np.random.uniform(28, 50)}.get(intensity, np.random.uniform(18, 38))

    acc[spike_at]   = mag
    acc[spike_at+1] = mag * 0.55
    acc[spike_at+2] = mag * 0.2
    acc = np.clip(acc, 4.0, None)   
    return acc, tilt

def gen_speed_bump(noise_sigma=None):
    if noise_sigma is None: noise_sigma = np.random.uniform(0.2, 0.5)
    N = 200
    acc  = GRAVITY + _noise(N, noise_sigma)
    tilt = 4.0 + _noise(N, 0.5)
    b0, bp, b1 = 30, 50, 70
    mag = np.random.uniform(12, 22)
    acc[b0:bp] = np.linspace(GRAVITY, mag, bp - b0)
    acc[bp:b1] = np.linspace(mag, GRAVITY, b1 - bp)
    acc = np.clip(acc, 4.0, None)
    return acc, tilt

def gen_hard_braking(noise_sigma=None):
    if noise_sigma is None: noise_sigma = np.random.uniform(0.2, 0.5)
    N = 200
    acc  = GRAVITY + _noise(N, noise_sigma)
    tilt = 6.0 + _noise(N, 0.8)
    b0, b1 = 30, 80
    peak = np.random.uniform(11, 15)
    acc[b0:b1]   = np.linspace(GRAVITY, peak, b1 - b0) + _noise(b1-b0, noise_sigma)
    acc[b1:b1+20] = np.linspace(peak, GRAVITY, 20) + _noise(20, noise_sigma)
    acc = np.clip(acc, 4.0, None)
    return acc, tilt

def gen_rough_road(surface="cobblestone"):
    N = 300
    vib_sigma = {"cobblestone": np.random.uniform(1.5, 3.0), "gravel": np.random.uniform(0.8, 1.8)}.get(surface, 1.5)
    acc  = GRAVITY + _noise(N, vib_sigma)
    tilt = 5.0 + _noise(N, 1.0)
    acc  = np.clip(acc, 4.5, 28.0)   
    return acc, tilt

def gen_curb_jump(noise_sigma=None):
    if noise_sigma is None: noise_sigma = np.random.uniform(0.2, 0.5)
    N = 250
    acc  = GRAVITY + _noise(N, noise_sigma)
    tilt = 5.0 + _noise(N, 0.5)

    air_start, air_end = 30, 44   
    impact    = air_end + 1

    ff_floor = np.random.uniform(1.0, 4.0)
    imp_mag  = np.random.uniform(20, 45)

    acc[air_start:air_end] = np.linspace(GRAVITY, ff_floor, air_end - air_start)
    acc[air_end]   = ff_floor
    acc[impact]    = imp_mag
    acc[impact+1]  = imp_mag * 0.5
    acc[impact+2:impact+6] = np.linspace(imp_mag * 0.3, GRAVITY, 4)

    acc[impact+6:]  = GRAVITY + _noise(N - impact - 6, noise_sigma * 0.3)
    tilt[impact+6:] = np.random.normal(7.0, 2.0, N - impact - 6)   

    acc[:air_start] = np.clip(acc[:air_start], 4.0, None)
    return acc, tilt


# trials

SCENARIOS = {
    "crash_lateral_slow":  (True,  lambda: gen_crash("lateral_slow")),
    "crash_lateral_fast":  (True,  lambda: gen_crash("lateral_fast")),
    "crash_forward":       (True,  lambda: gen_crash("forward")),
    "crash_soft_surface":  (True,  lambda: gen_crash("soft_surface")),
    "noncrash_pothole_mild":   (False, lambda: gen_pothole("mild")),
    "noncrash_pothole_medium": (False, lambda: gen_pothole("medium")),
    "noncrash_pothole_severe": (False, lambda: gen_pothole("severe")),
    "noncrash_speed_bump":     (False, lambda: gen_speed_bump()),
    "noncrash_hard_braking":   (False, lambda: gen_hard_braking()),
    "noncrash_cobblestone":    (False, lambda: gen_rough_road("cobblestone")),
    "noncrash_gravel":         (False, lambda: gen_rough_road("gravel")),
    "noncrash_curb_jump":      (False, lambda: gen_curb_jump()),
}

N_TRIALS = 500 # doing 6000 rn

def run_trials(impact_threshold=DEFAULT_IMPACT_THRESHOLD, jerk_threshold=DEFAULT_JERK_THRESHOLD, tilt_threshold=DEFAULT_TILT_THRESHOLD):
    it, jt, tt = impact_threshold, jerk_threshold, tilt_threshold
    out = {}
    for name, (is_crash, gen) in SCENARIOS.items():
        detections = []
        latencies  = []
        for _ in range(N_TRIALS):
            a, t = gen()
            r = run_fsm(a, t, it, jt, tt)
            detections.append(r.detected)
            if r.detected and r.detection_sample is not None:
                latencies.append(r.detection_sample * MS_PER_SAMPLE)
        nd = sum(detections)
        out[name] = {
            "is_crash":   is_crash,
            "n":          N_TRIALS,
            "n_detected": nd,
            "rate":       nd / N_TRIALS,
            "lat_mean":   float(np.mean(latencies)) if latencies else None,
            "lat_std":    float(np.std(latencies))  if latencies else None,
        }
    return out


def aggregate(results):
    tp = fp = tn = fn = 0
    for d in results.values():
        if d["is_crash"]: tp += d["n_detected"];  fn += d["n"] - d["n_detected"]
        else:             fp += d["n_detected"];  tn += d["n"] - d["n_detected"]
    return tp, fp, tn, fn


def metrics(tp, fp, tn, fn):
    total = tp + fp + tn + fn
    sens  = tp/(tp+fn)     if (tp+fn) > 0 else 0
    spec  = tn/(tn+fp)     if (tn+fp) > 0 else 0
    ppv   = tp/(tp+fp)     if (tp+fp) > 0 else 0
    f1    = 2*tp/(2*tp+fp+fn) if (2*tp+fp+fn) > 0 else 0
    acc   = (tp+tn)/total  if total > 0 else 0
    return dict(sensitivity=sens, specificity=spec, precision=ppv, f1=f1, accuracy=acc)


# more ML stuff 

def extract_features(acc, tilt):
    """Extracts stats from model"""
    return [
        float(np.min(acc)),          
        float(np.max(acc)),        
        float(np.std(acc)),          
        float(np.mean(tilt[-50:])),  
        float(np.var(acc[-50:]))     
    ]

def run_ml_pipeline():
    print("\n\n" + "="*60)
    print("running ML")
    print("="*60)
    
    X = []
    y = []
    
    print("generating dataset")
    for name, (is_crash, gen) in SCENARIOS.items():
        for _ in range(N_TRIALS):
            a, t = gen()
            X.append(extract_features(a, t))
            y.append(1 if is_crash else 0)
            
   
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
    
    print("Training Random Forest Classifier on {} samples...".format(len(X_train)))
    clf = RandomForestClassifier(n_estimators=100, random_state=42)
    clf.fit(X_train, y_train)
    
    print("Evaluating model on {} test samples...\n".format(len(X_test)))
    preds = clf.predict(X_test)
    
    print("Classification Report:")
    print(classification_report(y_test, preds, target_names=["Non-Crash (0)", "Crash (1)"]))
    
    print("Confusion Matrix:")
    cm = confusion_matrix(y_test, preds)
    print(f"True Negatives: {cm[0][0]}  | False Positives: {cm[0][1]}")
    print(f"False Negatives: {cm[1][0]}  | True Positives: {cm[1][1]}")


# main
if __name__ == "__main__":
    print("running main trials")
    main = run_trials()

    tp, fp, tn, fn = aggregate(main)
    m = metrics(tp, fp, tn, fn)

    print(f"\n FSM RESULTS (n={N_TRIALS}/scenario)")
    print(f"  TP={tp}  FP={fp}  TN={tn}  FN={fn}")
    for k, v in m.items():
        print(f"  {k:15s}: {v*100:.1f}%")

    print("\n FSM PER-SCENARIO")
    for name, d in main.items():
        lbl = "CRASH" if d["is_crash"] else "NON-CRASH"
        lat = f"{d['lat_mean']:.0f}±{d['lat_std']:.0f}ms" if d["lat_mean"] else "N/A"
        print(f"  [{lbl:9}] {name:35s}  {d['rate']*100:5.1f}%  lat={lat}")

    with open("sim_results.json", "w") as f:
        json.dump({"main": main, "aggregate": {"tp":tp,"fp":fp,"tn":tn,"fn":fn, **m}}, f, indent=2)
    print("\nSaved FSM results to sim_results.json")

   
    run_ml_pipeline()