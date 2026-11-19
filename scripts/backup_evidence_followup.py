"""Reuse the verified checkpoint copier for this bounded follow-up job."""
from pathlib import Path
import backup_targeted_comparison as copier

if __name__=='__main__':
    copier.OUT=Path(__file__).resolve().parents[1]/'outputs/evidence-followup-20261118/evidence'
    copier.REMOTE='/root/gpufree-data/supermimic_posture_width_20261117/outputs/evidence-followup-20261118'
    copier.main()
