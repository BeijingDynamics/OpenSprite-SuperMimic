"""Size in-memory TX evidence for the extended session and fix its end notice."""
from pathlib import Path


def patch(root):
    def edit(name, before, after):
        path = root / name
        text = path.read_text()
        if text.count(before) != 1:
            raise ValueError('Unexpected patch anchor: ' + name)
        path.write_text(text.replace(before, after), newline='\n')

    edit('native/src/action_tx_trace.hpp', 'events.reserve(200000)', 'events.reserve(600000)')
    edit('native/src/sling_window.hpp', 'constexpr double kXboxDurationSeconds = 146.0;',
         'constexpr double kXboxDurationSeconds = 146.0;\n'
         '// Round window close upward, then allow the full maximum braking time.\n'
         'constexpr int kXboxEndNoticeSeconds = (kXboxControlCloseFrame + 49) / 50 + 4;\n'
         'static_assert(kXboxEndNoticeSeconds == 145);')
    edit('native/src/sprite_can_shadow.cpp', 'if (seconds == 53) emit_status("XBOX_END\\n");',
         'if (seconds == sprite_policy::kXboxEndNoticeSeconds) emit_status("XBOX_END\\n");')
    edit('native/tests/test_action_tx.cpp', '  assert(rejected);\n  std::cout',
         '  assert(rejected);\n'
         '  // Exceed the old limit while retaining every packet identity without reallocation.\n'
         '  const auto* allocation = trace.events.data();\n'
         '  for (int i=2; i<500000; ++i) trace.sent(7, 30, 31);\n'
         '  assert(trace.events.data()==allocation && trace.events.size()==500000);\n'
         '  trace.stamp(499999, 600, 700);\n'
         '  assert(trace.events.back().hardware_raw_ns==700);\n'
         '  std::cout')


if __name__ == '__main__':
    patch(Path(__file__).resolve().parents[1] / 'outputs/deploy-sm-walk-r1-xbox120-20261119/source')
