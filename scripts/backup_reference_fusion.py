"""Preserve completed reference-fusion checkpoints and delivery artifacts locally."""
from pathlib import Path
import backup_targeted_comparison as copier

if __name__=='__main__':
    copier.OUT=Path(__file__).resolve().parents[1]/'outputs/reference-fusion-20261118/evidence'
    copier.REMOTE='/root/gpufree-data/supermimic_datafusion_20261118/outputs/reference-fusion-campaign'
    copier.main()
