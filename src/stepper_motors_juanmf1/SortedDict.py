from typing import MutableMapping


class SortedDict(MutableMapping):
    def __init__(self, *args, **kwargs):
        self._items = dict()
        self._keys = []

        if args or kwargs:
            self.update(*args, **kwargs)

    def __getitem__(self, key):
        return self._items[key]

    def __setitem__(self, key, value):
        if key not in self._items:
            self._keys.append(key)
            self._keys.sort()
        self._items[key] = value

    def __delitem__(self, key):
        del self._items[key]
        self._keys.remove(key)

    def __iter__(self):
        return iter(self._keys)

    def __len__(self):
        return len(self._items)

    def __repr__(self):
        return repr({k: self._items[k] for k in self._keys})

    def popitem(self, first=True):
        if not self._keys:
            raise KeyError('dictionary is empty')

        key = self._keys[0] if first else self._keys[-1]
        value = self._items.pop(key)
        self._keys.remove(key)
        return key, value
