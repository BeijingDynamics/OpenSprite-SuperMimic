"""Build an isolated echo-registration ordering fix; never load the module."""
from pathlib import Path
import difflib
import hashlib
import json
import shutil
import subprocess

s=Path('/home/tony/sprite-sm-walk-r1-fast-20261119')
old=Path('/home/tony/supermimic-run002-20261106/walk025-sync-1110/driver-tx-ioctl-v2')
new=s/'driver-echo-order'
new.mkdir(exist_ok=False)
for p in old.rglob('*'):
    if p.is_file() and (p.suffix in ('.c','.h') or p.name=='Makefile') and not p.name.endswith('.mod.c'):
        q=new/p.relative_to(old);q.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,q)
p=new/'src/kcan_netdev.c';before=p.read_text();after=before
anchor='\t/* put data into fifo */\n\terr = kcan_fifo_put(&pdev->tx_fifo, &f);'
assert after.count(anchor)==1
after=after.replace(anchor,'''	/* Register the echo skb before the frame becomes visible to a USB writer.
	 * A completion can run as soon as fifo_put publishes the frame. Registering
	 * after start_writing loses an early completion and can pair this skb with
	 * an unrelated completion when the 8-bit marker wraps. */
	if (tx_hwts) {
		err = KCAN_CAN_PUT_ECHO_SKB(skb, dev, mm);
		skb_consumed = true;
		if (err) {
			stats->tx_dropped++;
			goto free_out;
		}
		echo_stored = true;
	}

'''+anchor)
anchor='\t\tstats->tx_fifo_errors++; /* just for informational purposes */'
assert after.count(anchor)==1
after=after.replace(anchor,anchor+'''

		/* The frame was not published. Release the registered echo exactly once;
		 * free_out releases the reserved marker, not the already consumed skb. */
		if (echo_stored) {
			KCAN_CAN_FREE_ECHO_SKB(dev, mm);
			echo_stored = false;
		}''')
anchor='''	if (tx_hwts) {
		KCAN_CAN_PUT_ECHO_SKB(skb, dev, mm);
		echo_stored = true;
		/* Hardware timestamp will be set in TX complete interrupt handler
		 * via skb_hwtstamps(tx_skb)->hwtstamp, then can_get_echo_skb()
		 * will notify the socket.
		 */
	} else if (priv->can.echo_skb_max) {'''
assert after.count(anchor)==1
after=after.replace(anchor,'''	if (tx_hwts) {
		/* The echo was registered before publication; completion owns it now. */
	} else if (priv->can.echo_skb_max) {''')
assert after.index('err = KCAN_CAN_PUT_ECHO_SKB(skb, dev, mm);')<after.index('err = kcan_fifo_put(&pdev->tx_fifo, &f);')
p.write_text(after)
(s/'driver-echo-order.patch').write_text(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='a/src/kcan_netdev.c',tofile='b/src/kcan_netdev.c')))
with (s/'driver-echo-order-build.log').open('w') as log:
    subprocess.run(['make','-j2','netdev'],cwd=new,stdout=log,stderr=subprocess.STDOUT,check=True)
sha=hashlib.sha256((new/'kcan.ko').read_bytes()).hexdigest()
(s/'driver-echo-order.sha256').write_text(sha+'  driver-echo-order/kcan.ko\n')
manifest={str(p.relative_to(new)):hashlib.sha256(p.read_bytes()).hexdigest() for p in new.rglob('*') if p.is_file() and p.suffix in ('.c','.h') and not p.name.endswith('.mod.c')}
(s/'driver-echo-order-manifest.json').write_text(json.dumps(dict(module_sha256=sha,source=manifest),indent=2)+'\n')
print('Built isolated echo-order module; not loaded. SHA256 '+sha)
