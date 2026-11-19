export default {
  extends: ['@commitlint/config-conventional'],
  rules: {
    'type-enum': [
      2,
      'always',
      ['feat', 'fix', 'perf', 'refactor', 'test', 'docs', 'ci', 'build', 'chore', 'revert', 'style'],
    ],
    'scope-empty': [0], // scope optional
    'scope-case': [0], // scopes are not enforced
    'subject-case': [0],
    'header-max-length': [2, 'always', 100],
    'body-max-line-length': [0],
    'footer-max-line-length': [0],
  },
  ignores: [(msg) => /^chore\(release\): /.test(msg)],
};
